"""The automatic per-session checks behind tools/check_session.py.

Each check returns Result rows: what was checked, for which robot / run /
recording, PASS / CHECK / FAIL (or SKIP when the data isn't there, INFO for a
number that is only reported), the number, and the pass rule. check_session.py
prints them as plain sentences and writes them all to
<session>/session_checks.csv (format: docs/data-formats.md).

Grading (unless a check says otherwise): a number under its limit is PASS,
up to twice the limit is CHECK (look at it before trusting the data), more is
FAIL.

Only the standard library is needed for the robot-log checks; the video
checks need numpy and opencv-python.
"""
import csv
import json
import math
import os
import statistics as st
import sys
from dataclasses import dataclass

if os.path.dirname(os.path.abspath(__file__)) not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import raw_units as RU  # noqa: E402  (RAW v_cms is mm/s; rpm, v_cms, w_dps read 1.08 high)

CSV_COLS = ["check", "scope", "status", "value", "unit", "rule", "detail"]

# The fusion's "still" test (tools/fusion/engine.py): the wheels say the robot isn't moving. True units (rpm,
# cm/s, deg/s): the RAW columns go through tools/raw_units.py first.
FUSION_STILL = dict(rpm=0.2, v_cms=0.1, w_dps=0.5)
_TRUE = dict(wl_rpm=RU.wheel_rpm, wr_rpm=RU.wheel_rpm, v_cms=RU.speed_cms, w_dps=RU.turn_dps)


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


@dataclass
class Result:
    check: str      # short id, e.g. video_clock_drift
    title: str      # plain words, e.g. "Video clock drift"
    scope: str      # which robot / run / recording
    status: str     # PASS, CHECK, FAIL, SKIP, INFO
    value: float    # the number the rule is about (nan if none)
    unit: str
    rule: str       # the pass rule, plain words
    detail: str     # the sentence printed

    def line(self):
        return f"{self.status:<5s}  {self.title} ({self.scope}): {self.detail}" + (
            f"  [pass: {self.rule}]" if self.rule else "")

    def row(self):
        v = "" if self.value != self.value else f"{self.value:.6g}"
        return [self.check, self.scope, self.status, v, self.unit, self.rule, self.detail]


# A calibration run's outcome (docs/data-formats.md, "Calibration outcomes"): three states.
CALIB_DONE = "done"          # everything it tries was done
CALIB_LIMITED = "limited"    # RUNEND done, but it skipped or cut short the spin or the roll: no room, or a plywood edge
CALIB_FAILED = "failed"      # RUNEND failed:... or stopped:..., or no RUNEND at all
CALIB_LIMIT_REASONS = ("no room", "plywood edge")   # CALIB_LOOK reason parts ("; " between) that mean "limited"


# Any run's RUNEND result (firmware 2026-10-18c+): "done", "limited:plywood edge" (the edge guard
# refused to drive off the work area: not a failure, but the run did not finish its route, so accuracy
# statistics leave it out), "stopped:<why>", "failed:<why>".
RUN_LIMITED_PREFIX = "limited:"
LIMITED_EDGE_NOTE = "limited: plywood edge"   # run_sheet.py's note on such a row (tape_results.is_limited)


def run_outcome(result):
    """CALIB_DONE / CALIB_LIMITED / CALIB_FAILED for any run's RUNEND result (None = no RUNEND)."""
    r = (result or "").strip()
    if r == "done":
        return CALIB_DONE
    if r.startswith(RUN_LIMITED_PREFIX):
        return CALIB_LIMITED
    return CALIB_FAILED


def run_limit_reason(result):
    """"plywood edge" for "limited:plywood edge", else ""."""
    r = (result or "").strip()
    return r[len(RUN_LIMITED_PREFIX):].strip() if r.startswith(RUN_LIMITED_PREFIX) else ""


def calib_outcome(result, look=None):
    """The outcome of a calibration run (CALIB_DONE / CALIB_LIMITED / CALIB_FAILED) from its RUNEND
    result and its CALIB_LOOK record (a dict with the record's column names; None for firmware
    before 2026-10-18c). Skipping the spin or the roll for lack of room, or the edge guard stopping
    the roll at a plywood edge, is not a failure: the robot refused something it had no room for or
    that was unsafe, so it is its own state, CALIB_LIMITED."""
    if run_outcome(result) == CALIB_LIMITED:
        return CALIB_LIMITED               # an edge-guard stop outside the forward roll (none expected)
    if (result or "").strip() != "done":
        return CALIB_FAILED
    reasons = calib_limit_reasons(look)
    if reasons:
        return CALIB_LIMITED
    return CALIB_DONE


def calib_limit_reasons(look):
    """The CALIB_LOOK reason as a list (e.g. ["no room", "plywood edge"]); [] = nothing limited."""
    look = look or {}
    reason = (look.get("reason") or "").strip()
    if reason:
        return [r.strip() for r in reason.split(";") if r.strip() in CALIB_LIMIT_REASONS]
    if "reason" not in look and (look.get("result") or "").strip() in ("no_room", "no_room_after_turn"):
        return ["no room"]                 # a CALIB_LOOK without the spin/roll/reason columns
    return []


def calib_outcome_text(result, look=None):
    """calib_outcome() for people and calibration_results.csv: done, failed, or "limited (<reason>)",
    e.g. "limited (no room)", "limited (plywood edge)", "limited (no room; plywood edge)"."""
    oc = calib_outcome(result, look)
    if oc != CALIB_LIMITED:
        return oc
    reasons = calib_limit_reasons(look) or [run_limit_reason(result)]
    return "%s (%s)" % (oc, "; ".join(reasons))


def grade(x, limit, fail_factor=2.0):
    """PASS below limit, CHECK below fail_factor x limit, else FAIL (nan: SKIP)."""
    if x is None or x != x:
        return "SKIP"
    return "PASS" if x < limit else ("CHECK" if x < fail_factor * limit else "FAIL")


def worst(*statuses):
    order = ["SKIP", "INFO", "PASS", "CHECK", "FAIL"]
    s = [x for x in statuses if x in order]
    return max(s, key=order.index) if s else "SKIP"


def write_csv(path, results):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLS)
        for r in results:
            w.writerow(r.row())


def linfit(xs, ys):
    """Least squares y = a + b x. Returns (a, b, residuals, slope standard error)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0 else 0.0
    a = my - b * mx
    res = [y - (a + b * x) for x, y in zip(xs, ys)]
    se = math.sqrt(sum(r * r for r in res) / (n - 2) / sxx) if n > 2 and sxx > 0 else float("nan")
    return a, b, res, se


# ================================================================== robot logs
def sorted_rows(rr):
    return sorted(rr, key=lambda r: fnum(r.get("ms")))


def still_windows(rr, min_s=0.5, trim_s=0.2, max_steps_deg=0.72, max_gap_s=0.5):
    """Stretches of a run's RAW rows (sorted by ms) where the robot sits still:
    both wheel angles stay within max_steps_deg (2 encoder steps of 0.36 deg),
    not lifted, no gap over max_gap_s. trim_s is cut from each end (the robot
    settles after braking), and what is left must last min_s.
    The wheel ANGLES decide, not the rpm columns: rpm noise is one of the
    things being checked. Returns [(i0, i1)] row index ranges, i1 inclusive."""
    out = []
    n = len(rr)
    ms = [fnum(r.get("ms")) for r in rr]
    wl = [fnum(r.get("wl_deg")) for r in rr]
    wr = [fnum(r.get("wr_deg")) for r in rr]
    lifted = [fnum(r.get("lifted"), 0) > 0.5 for r in rr]

    def close(i0, i1):
        a, b = ms[i0] + trim_s * 1000, ms[i1] - trim_s * 1000
        if b - a >= min_s * 1000:
            j0 = next(k for k in range(i0, i1 + 1) if ms[k] >= a)
            j1 = max(k for k in range(i0, i1 + 1) if ms[k] <= b)
            out.append((j0, j1))

    i = 0
    while i < n:
        if lifted[i] or wl[i] != wl[i] or wr[i] != wr[i]:
            i += 1
            continue
        lo_l = hi_l = wl[i]
        lo_r = hi_r = wr[i]
        j = i
        while j + 1 < n:
            k = j + 1
            if lifted[k] or (ms[k] - ms[j]) > max_gap_s * 1000 or wl[k] != wl[k] or wr[k] != wr[k]:
                break
            nl0, nl1 = min(lo_l, wl[k]), max(hi_l, wl[k])
            nr0, nr1 = min(lo_r, wr[k]), max(hi_r, wr[k])
            if nl1 - nl0 > max_steps_deg or nr1 - nr0 > max_steps_deg:
                break
            lo_l, hi_l, lo_r, hi_r = nl0, nl1, nr0, nr1
            j = k
        close(i, j)
        i = j + 1
    return out


def wifi_checks(key, rr, sent, max_gap_ms=300.0, max_loss_pct=2.0):
    """Check 7: how the RAW rows that never arrived are spread. A burst of
    consecutive lost rows is a hole in the record; scattered single losses
    barely matter. The library's move()/rotate() calls stop the robot's loop
    for ~100-200 ms by design (no rows are made, none lost): a 90-300 ms gap
    that starts while the wheels are still is counted as such a pause, not
    a loss. RAW has no row counter, so this split is an estimate; the loss
    percentage (rows sent vs received) is exact."""
    robot, run = key
    rs = [r for r in sorted_rows(rr) if fnum(r.get("ms")) == fnum(r.get("ms"))]
    ms = [fnum(r.get("ms")) for r in rs]
    scope = f"{robot} run {run}"
    if len(ms) < 3:
        return [Result("wifi_loss", "WiFi loss in the sensor log", scope, "SKIP", float("nan"), "", "",
                       "fewer than 3 RAW rows")]
    iv = [b - a for a, b in zip(ms, ms[1:])]
    med = st.median(iv)
    wl = [fnum(r.get("wl_deg"), 0) for r in rs]
    wr = [fnum(r.get("wr_deg"), 0) for r in rs]
    lost_runs, pauses = [], 0
    for i, g in enumerate(iv):
        m = max(0, round(g / med) - 1) if med > 0 else 0
        if m == 0:
            continue
        still_before = i >= 1 and abs(wl[i] - wl[i - 1]) <= 0.72 and abs(wr[i] - wr[i - 1]) <= 0.72
        if 90 <= g <= 300 and still_before:
            pauses += 1
            continue
        lost_runs.append(m)
    bursts = [m for m in lost_runs if m >= 2]
    longest_run = max(lost_runs, default=0)
    gap = max(iv)
    got = len(ms)
    loss = 100 * (1 - got / sent) if sent else float("nan")
    st_gap = grade(gap, max_gap_ms, 1.5)
    st_loss = grade(loss, max_loss_pct) if loss == loss else "SKIP"
    status = worst(st_gap, st_loss) if st_loss != "SKIP" else st_gap
    detail = (f"{got} of {sent or '?'} rows arrived ({loss:.1f}% lost); longest gap {gap:.0f} ms "
              f"(rows normally {med:.0f} ms apart; {pauses} gap(s) look like the library's planned pauses); "
              f"outside those, longest run of lost rows {longest_run}, {len(bursts)} burst(s) of 2+")
    out = [Result("wifi_loss", "WiFi loss in the sensor log", scope, status, gap, "ms",
                  f"longest gap < {max_gap_ms:.0f} ms and loss < {max_loss_pct:g}%", detail)]
    hist = {"1 row": 0, "2-3 rows": 0, "4-9 rows": 0, "10+ rows": 0}
    for m in lost_runs:
        hist["1 row" if m == 1 else "2-3 rows" if m <= 3 else "4-9 rows" if m <= 9 else "10+ rows"] += 1
    out.append(Result("wifi_bursts", "WiFi loss bursts", scope, "INFO", float(longest_run), "rows", "",
                      "holes in the log (not counting planned pauses) by size: "
                      + ", ".join(f"{k}: {v}" for k, v in hist.items())))
    return out


def clock_checks(fits, max_resid_ms=5.0, max_rtt_ms=50.0, max_ppm=200.0):
    """Check 8: robot clock -> laptop clock fit, per robot per boot."""
    out = []
    for (robot, boot), (a, b, i) in sorted(fits.items()):
        scope = f"{robot} boot {boot or '?'}"
        measured = i.get("used", 0) >= 2 and b != 1.0
        s_res = grade(i["resid_ms"], max_resid_ms)
        s_rtt = grade(i["rtt_med"], max_rtt_ms)
        s_ppm = grade(abs(i["drift_ppm"]), max_ppm) if measured else "CHECK"
        status = worst(s_res, s_rtt, s_ppm)
        detail = (f"{i['used']} of {i['bursts']} clock-check bursts used; worst fit error {i['resid_ms']:.1f} ms; "
                  f"round trip median {i['rtt_med']:.1f} ms; robot clock "
                  + (f"{abs(i['drift_ppm']):.0f} ppm {'fast' if i['drift_ppm'] >= 0 else 'slow'}" if measured
                     else "drift NOT measured (clock checks span under 5 s: rate assumed exact)"))
        out.append(Result("robot_clock", "Robot clock fit", scope, status, i["resid_ms"], "ms",
                          f"fit error < {max_resid_ms:g} ms, round trip < {max_rtt_ms:g} ms, "
                          f"drift < {max_ppm:g} ppm", detail))
    return out


def _rest_rows(raw_by_run):
    """{robot: [(run, rows of one still window)]}, plus {robot: boot ids}."""
    out = {}
    for (robot, run), rr in raw_by_run.items():
        rs = sorted_rows(rr)
        for i0, i1 in still_windows(rs):
            out.setdefault(robot, []).append((run, rs[i0:i1 + 1]))
    return out


def tilt_checks(raw_by_run, max_tilt_deg=2.0, az_tol_g=0.05):
    """Check 9: floor level (and accelerometer units): the accelerometer at
    rest points along gravity; its angle from the robot's vertical is the tilt
    of the floor under the robot PLUS the sensor's own mounting/bias error
    (the two can't be told apart from one robot: see the report notes)."""
    out = []
    for robot, wins in sorted(_rest_rows(raw_by_run).items()):
        tilts, azs = [], []
        for run, w in wins:
            ax = st.mean(fnum(r.get("ax")) for r in w)
            ay = st.mean(fnum(r.get("ay")) for r in w)
            az = st.mean(fnum(r.get("az")) for r in w)
            if az == az and ax == ax and ay == ay:
                tilts.append(math.degrees(math.atan2(math.hypot(ax, ay), az)))
                azs.append(math.sqrt(ax * ax + ay * ay + az * az))
        if not tilts:
            continue
        med, mx = st.median(tilts), max(tilts)
        out.append(Result("tilt", "Floor level (accelerometer tilt at rest)", robot, grade(mx, max_tilt_deg), mx,
                          "deg", f"tilt < {max_tilt_deg:g} deg",
                          f"tilt {med:.2f} deg typical, {mx:.2f} deg worst, over {len(tilts)} still period(s) "
                          f"(spread {mx - min(tilts):.2f} deg: a spread means the floor isn't flat everywhere)"))
        g = st.median(azs)
        out.append(Result("gravity", "Accelerometer reads 1 g at rest", robot, grade(abs(g - 1), az_tol_g), g, "g",
                          f"within {az_tol_g:g} g of 1", f"total acceleration at rest {g:.3f} g "
                          "(about 1.00 means the columns are in g)"))
    return out


def rest_noise_checks(raw_by_run, still=FUSION_STILL, min_share=0.95):
    """Check 10: while the wheels truly don't turn, do the rpm / speed / turn
    rate columns stay under the fusion's 'still' thresholds? If not, the fusion
    misses still periods (no gyro-bias update)."""
    out = []
    for robot, wins in sorted(_rest_rows(raw_by_run).items()):
        rows = [r for _, w in wins for r in w]
        if not rows:
            continue

        def tv(r, col):
            return _TRUE[col](r.get(col))

        def p95(col):
            v = sorted(abs(tv(r, col)) for r in rows if tv(r, col) == tv(r, col))
            return v[int(0.95 * (len(v) - 1))] if v else float("nan")
        ok = sum(1 for r in rows if abs(tv(r, "wl_rpm")) < still["rpm"] and abs(tv(r, "wr_rpm")) < still["rpm"]
                 and abs(tv(r, "v_cms")) < still["v_cms"] and abs(tv(r, "w_dps")) < still["w_dps"])
        share = ok / len(rows)
        rpm = max(p95("wl_rpm"), p95("wr_rpm"))
        v, w = p95("v_cms"), p95("w_dps")
        status = "PASS" if share >= min_share else ("CHECK" if share >= 0.5 else "FAIL")
        out.append(Result("rest_noise", "Wheel readings at rest vs the fusion's 'still' test", robot, status,
                          100 * share, "%", f"at least {100 * min_share:.0f}% of still rows pass the fusion's test "
                          f"(rpm < {still['rpm']:g}, speed < {still['v_cms']:g} cm/s, turn < {still['w_dps']:g} deg/s)",
                          f"{100 * share:.0f}% of {len(rows)} rows with the wheels not turning would count as still; "
                          f"95% of rest readings are within rpm {rpm:.2f}, speed {v:.2f} cm/s, turn {w:.2f} deg/s"))
    return out


def gyro_bias_checks(raw_by_run, max_change_dps=0.2):
    """Check 11: gyro bias (the reading at rest) per still period, against
    time since power-on: a bias that changes after boot is warm-up drift the
    fusion has to keep re-learning."""
    out = []
    per = {}
    for robot, wins in _rest_rows(raw_by_run).items():
        for run, w in wins:
            boot = run.split("-")[0]
            t = st.mean(fnum(r.get("ms")) for r in w) / 1000
            per.setdefault((robot, boot), []).append((t, *(st.mean(fnum(r.get(c)) for r in w) for c in ("gx", "gy", "gz"))))
    for (robot, boot), pts in sorted(per.items()):
        pts.sort()
        scope = f"{robot} boot {boot}"
        if len(pts) < 2:
            out.append(Result("gyro_bias", "Gyro bias change at rest (warm-up)", scope, "SKIP", float("nan"), "deg/s",
                              f"change < {max_change_dps:g} deg/s", f"only {len(pts)} still period(s)"))
            continue
        ch = max(abs(pts[-1][k] - pts[0][k]) for k in (1, 2, 3))
        rng = max(max(p[k] for p in pts) - min(p[k] for p in pts) for k in (1, 2, 3))
        dz = pts[-1][3] - pts[0][3]
        out.append(Result("gyro_bias", "Gyro bias change at rest (warm-up)", scope, grade(ch, max_change_dps), ch,
                          "deg/s", f"first-to-last change < {max_change_dps:g} deg/s",
                          f"{len(pts)} still periods from {pts[0][0] / 60:.1f} to {pts[-1][0] / 60:.1f} min after "
                          f"power-on; yaw gyro bias {pts[0][3]:+.3f} -> {pts[-1][3]:+.3f} deg/s ({dz:+.3f}); "
                          f"largest change on any axis {ch:.3f} deg/s, largest range {rng:.3f} deg/s"))
    return out


def lifted_checks(raw_by_run):
    """Check 12: the robot reported being picked up during a run."""
    out = []
    for (robot, run), rr in sorted(raw_by_run.items()):
        if not rr or "lifted" not in rr[0]:
            out.append(Result("lifted", "Robot picked up during the run", f"{robot} run {run}", "SKIP", float("nan"),
                              "rows", "no lifted rows", "no lifted column (firmware before 2026-10-17c)"))
            continue
        n = sum(1 for r in rr if fnum(r.get("lifted"), 0) > 0.5)
        out.append(Result("lifted", "Robot picked up during the run", f"{robot} run {run}",
                          "PASS" if n == 0 else "CHECK", float(n), "rows", "no lifted rows",
                          "never lifted" if n == 0 else
                          f"{n} rows with lifted = 1 ({n / len(rr) * 100:.1f}% of the run): picked up, bumped, "
                          "or a wheel off the floor?"))
    return out


TOF_ZONES = ["dL_cm", "dCL_cm", "dC_cm", "dCR_cm", "dR_cm", "dtop_cm", "dbot_cm"]


def tof_checks(raw_by_run, min_fps=10.0, max_age_ms=200.0):
    """Check 13: the front distance sensor makes fewer frames than RAW rows;
    RAW repeats the last frame. A new frame = any zone changed. Reports
    frames per second, rows per frame, and the age of the reading (time since
    the frame changed) per robot."""
    out = []
    by_robot = {}
    for (robot, run), rr in raw_by_run.items():
        by_robot.setdefault(robot, []).append(sorted_rows(rr))
    for robot, runs in sorted(by_robot.items()):
        cols = [c for c in TOF_ZONES if runs and runs[0] and c in runs[0][0]]
        if not cols:
            continue
        n_new, secs, reps, ages, per_zone = 0, 0.0, [], [], {c: 0 for c in cols}
        for rs in runs:
            if len(rs) < 3:
                continue
            ms = [fnum(r.get("ms")) for r in rs]
            secs += (ms[-1] - ms[0]) / 1000
            prev = tuple(rs[0].get(c) for c in cols)
            t_new, run_len = ms[0], 1
            for r, m in zip(rs[1:], ms[1:]):
                cur = tuple(r.get(c) for c in cols)
                for k, c in enumerate(cols):
                    if cur[k] != prev[k]:
                        per_zone[c] += 1
                if cur != prev:
                    n_new += 1
                    reps.append(run_len)
                    run_len = 1
                    t_new = m
                else:
                    run_len += 1
                ages.append(m - t_new)
                prev = cur
        if secs <= 0:
            continue
        fps = n_new / secs
        rep = st.median(reps) if reps else float("nan")
        age_med = st.median(ages) if ages else float("nan")
        age_max = max(ages) if ages else float("nan")
        status = worst("PASS" if fps >= min_fps else ("CHECK" if fps >= min_fps / 2 else "FAIL"),
                       "PASS" if age_max <= max_age_ms else "CHECK")
        out.append(Result("tof_rate", "Front distance sensor frame rate", robot, status, fps, "frames/s",
                          f">= {min_fps:g} new frames/s and no reading older than {max_age_ms:.0f} ms",
                          f"{fps:.1f} new frames a second; each frame repeated over {rep:.0f} RAW row(s) typically; "
                          f"reading age {age_med:.0f} ms typical, {age_max:.0f} ms worst; changes per zone per s: "
                          + ", ".join(f"{c[:-3]} {per_zone[c] / secs:.1f}" for c in cols)
                          + " (a zone that never changes may be out of range: then frames are undercounted)"))
    return out


# ================================================================== video
def read_meta(rec):
    try:
        with open(os.path.join(rec, "meta.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def nominal_fps(rec, meta=None):
    """The frame rate the camera was set to: meta.json fps_reported, else
    fps_requested, else None."""
    m = meta if meta is not None else read_meta(rec)
    for k in ("fps_reported", "fps_requested"):
        v = fnum(m.get(k))
        if v == v and v > 0:
            return v
    return None


def read_frames_csv(rec):
    """(t_wall list, cam_ms list or None)."""
    tw, cm = [], []
    with open(os.path.join(rec, "frames.csv"), newline="") as f:
        rd = csv.DictReader(f)
        has_cam = "cam_ms" in (rd.fieldnames or [])
        for r in rd:
            tw.append(fnum(r.get("t_wall")))
            if has_cam:
                cm.append(fnum(r.get("cam_ms")))
    return tw, (cm if has_cam else None)


def frame_timing_checks(rec, max_drop_pct=0.5, max_jitter_ms=5.0, fps_tol_pct=2.0):
    """Check 4: dropped frames, frame rate vs the setting, and the delivery
    jitter (spread of when frames reach the Pi compared with when the camera
    took them)."""
    name = os.path.basename(os.path.normpath(rec))
    out = []
    tw, cam = read_frames_csv(rec)
    tw = [t for t in tw if t == t]
    if len(tw) < 10:
        return [Result("frame_drops", "Dropped camera frames", name, "SKIP", float("nan"), "", "", "too few frames")]
    iv = [b - a for a, b in zip(tw, tw[1:])]
    med = st.median(iv)
    # A frame that reached the Pi late is followed by one that arrives almost at
    # once: that is lateness, not a drop. Pull late frames back (as
    # blink.repair_late_frames does) before counting the frames missing in gaps.
    rep = list(tw)
    for i in range(len(rep) - 2, 0, -1):
        if rep[i + 1] - rep[i] < 0.5 * med:
            rep[i] = min(rep[i], rep[i + 1] - med)
    steps = [max(1, round((y - x) / med)) for x, y in zip(rep, rep[1:])]
    dropped = sum(s - 1 for s in steps)
    late = sum(1 for x in iv if x > 1.5 * med)
    pct = 100 * dropped / (len(tw) + dropped)
    out.append(Result("frame_drops", "Dropped camera frames", name, grade(pct, max_drop_pct), pct, "%",
                      f"< {max_drop_pct:g}% of frames dropped",
                      f"{dropped} frame(s) missing of {len(tw) + dropped} ({pct:.2f}%); {late} gap(s) over 1.5 frames "
                      f"(the rest were frames that arrived late); "
                      f"longest gap {max(iv) * 1000:.0f} ms"))
    fps = 1 / med
    nom = nominal_fps(rec)
    if nom:
        dev = 100 * abs(fps - nom) / nom
        out.append(Result("frame_rate", "Camera frame rate vs its setting", name, grade(dev, fps_tol_pct), fps, "fps",
                          f"within {fps_tol_pct:g}% of the {nom:g} fps setting",
                          f"{fps:.2f} fps (frame spacing {med * 1000:.1f} ms); set to {nom:g} fps"))
    else:
        out.append(Result("frame_rate", "Camera frame rate vs its setting", name, "INFO", fps, "fps", "",
                          f"{fps:.2f} fps; meta.json doesn't say what it was set to"))
    # jitter: arrival time minus capture time, after removing a straight line
    # (the two clocks differ by an offset and a rate)
    idx = None
    src = ""
    if cam and len(cam) == len(tw) + 0 and sum(1 for c in cam if c == c and c > 0) > 0.9 * len(cam):
        pairs = [(c / 1000, t) for c, t in zip(cam, tw) if c == c and c > 0 and t == t]
        cs_ = [p[0] for p in pairs]
        if len(set(cs_)) > 0.9 * len(cs_) and all(b >= a for a, b in zip(cs_, cs_[1:])):
            xs, ys = cs_, [p[1] for p in pairs]
            src = "camera driver timestamps (cam_ms)"
            idx = True
    if idx is None:
        # no usable cam_ms: the camera's own clock ticks evenly, so frame k was taken at
        # t0 + k x period; count missing frames into k
        k, xs = 0, [0.0]
        for s in steps:
            k += s
            xs.append(float(k))
        ys = tw
        src = "an even frame grid (no camera timestamps: a proxy)"
    a, b, res, _ = linfit(xs, ys)
    sd = st.pstdev(res) * 1000
    srt = sorted(abs(r) for r in res)
    p99 = srt[int(0.99 * (len(srt) - 1))] * 1000
    out.append(Result("frame_jitter", "Frame delivery jitter", name, grade(sd, max_jitter_ms), sd, "ms",
                      f"SD < {max_jitter_ms:g} ms",
                      f"arrival time scatters {sd:.1f} ms (SD) around {src}; 99% within {p99:.1f} ms, worst "
                      f"{max(srt) * 1000:.0f} ms"))
    return out


def camera_settings_check(rec):
    """Check 3b + 6: exposure locked, and the camera's settings the same at
    the end as at the start (camrec.py meta.json)."""
    name = os.path.basename(os.path.normpath(rec))
    m = read_meta(rec)
    out = []
    e = fnum(m.get("exposure_ms"))
    out.append(Result("exposure", "Exposure locked and known", name, "PASS" if e == e and e > 0 else "CHECK",
                      e, "ms", "exposure_ms in meta.json (locked)",
                      f"exposure {e:g} ms (from meta.json; flash timing uses it)" if e == e and e > 0 else
                      "exposure not locked or not recorded: flash timing assumes the exposure fills the frame "
                      "and blink.py assumes 12 ms"))
    cc = m.get("camera_controls") or {}
    start = cc.get("after") if isinstance(cc, dict) else None
    end = m.get("camera_controls_end")
    if not start or not end:
        out.append(Result("camera_settings", "Camera settings unchanged start to end", name, "SKIP", float("nan"), "",
                          "same at start and end", "meta.json lacks camera_controls / camera_controls_end "
                          "(recorded before 2026-10-18, or the recorder didn't stop cleanly)"))
        return out
    changed = []
    for k in sorted(set(start) | set(end)):
        a, b = fnum(start.get(k)), fnum(end.get(k))
        if a != a and b != b:
            continue
        if a != a or b != b or abs(a - b) > max(1e-6, 0.01 * abs(a)):
            changed.append(f"{k} {start.get(k)} -> {end.get(k)}")
    out.append(Result("camera_settings", "Camera settings unchanged start to end", name,
                      "PASS" if not changed else "CHECK", float(len(changed)), "settings", "same at start and end",
                      "all the same" if not changed else "changed: " + "; ".join(changed)))
    return out


def video_clock_check(name, pairs, max_drift_ms=5.0, max_sd_ms=10.0, write_to=None, how="switch-on times",
                      level_pairs=None):
    """Check 1: offset (video clock - laptop clock) at each sync flash against
    time. Fits offset = offset_s + drift x (t - t_ref). Writes
    <recording>/video_clock.json with the line (format: docs/data-formats.md)
    when write_to is given and 2+ flashes were matched. level_pairs: other
    (t, offset) pairs that set the line's level (offset_s) while `pairs` set
    its slope and scatter, so the file's offset keeps the timing convention
    of check_session's switch-on times (which tools/fusion uses)."""
    pairs = sorted(pairs)
    if len(pairs) < 2:
        return [Result("video_clock_drift", "Video clock drift vs laptop", name, "SKIP", float("nan"), "ms",
                       f"drift adds < {max_drift_ms:g} ms and scatter < {max_sd_ms:g} ms",
                       f"only {len(pairs)} sync flash(es) matched in the video: need 2+ (3+ for the scatter)")]
    med = st.median(o for _, o in pairs)
    kept = [(t, o) for t, o in pairs if abs(o - med) <= 0.05]       # same odd-one-out rule as fusion/runs.py
    if len(kept) < 2:
        kept = pairs
    ts, os_ = [t for t, _ in kept], [o for _, o in kept]
    t_ref = st.mean(ts)
    span = max(ts) - min(ts)
    a, b, res, se = linfit([t - t_ref for t in ts], os_)
    if level_pairs:
        lv = [o - b * (t - t_ref) for t, o in level_pairs]
        lm = st.median(lv)
        lv = [x for x in lv if abs(x - lm) <= 0.05]
        a = st.mean(lv) if lv else a
    contrib = abs(b) * span * 1000
    sd = math.sqrt(sum(r * r for r in res) / (len(res) - 2)) * 1000 if len(res) > 2 else float("nan")
    real = bool(len(res) > 2 and se == se and abs(b) > 3 * se)
    s1 = grade(contrib, max_drift_ms)
    if s1 != "PASS" and not real:
        s1 = "CHECK"        # the slope is within the scatter: drift can't be confirmed under the limit, nor measured
    s2 = grade(sd, max_sd_ms) if sd == sd else "CHECK"
    status = worst(s1, s2)
    detail = (f"{len(kept)} flashes ({how}) over {span / 60:.1f} min; video clock {'gains' if b >= 0 else 'loses'} "
              f"{abs(b) * 1e6:.1f} ppm on the laptop ({contrib:.1f} ms over the session"
              + (", measurably" if real else ", not distinguishable from scatter" if len(res) > 2 else "")
              + f"); scatter around the line {sd:.1f} ms (SD)" if sd == sd else
              f"{len(kept)} flashes: drift {b * 1e6:.1f} ppm ({contrib:.1f} ms), scatter can't be measured from 2")
    if len(kept) < len(pairs):
        detail += f"; {len(pairs) - len(kept)} flash(es) more than 50 ms off left out"
    if write_to:
        with open(os.path.join(write_to, "video_clock.json"), "w") as f:
            json.dump(dict(format="video-clock", version=1, offset_s=a, t_ref=t_ref, drift_ppm=b * 1e6,
                           drift_significant=real, use_drift=bool(real and contrib >= 1.0), resid_sd_ms=sd,
                           n_flashes=len(kept), span_s=span, from_edges=how,
                           note="video t_wall = laptop time + offset_s + drift_ppm * 1e-6 * (laptop time - t_ref); "
                                "written by tools/check_session.py"), f, indent=1)
        detail += "; line written to video_clock.json"
    return [Result("video_clock_drift", "Video clock drift vs laptop", name, status, contrib, "ms",
                   f"drift adds < {max_drift_ms:g} ms and scatter < {max_sd_ms:g} ms", detail)]


# ---------------------------------------------------------------- image registration
def _gray(img):
    import cv2
    import numpy as np
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    return g.astype(np.float32)


def read_window(cap, start, n):
    """n frames from frame `start` as a float32 stack (fewer at the end)."""
    import cv2
    import numpy as np
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(start))
    out = []
    for _ in range(n):
        ok, img = cap.read()
        if not ok:
            break
        out.append(_gray(img))
    return np.stack(out) if out else None


def static_view(stack):
    """(median image, mask of pixels that didn't change in the window).
    Moving robots / hands are left out by frame differencing against the
    window's median; the mask is grown a little around them."""
    import cv2
    import numpy as np
    med = np.median(stack, axis=0).astype(np.float32)
    dev = np.abs(stack - med[None])
    noise = float(np.median(dev)) * 1.4826 + 0.5
    moving = (dev > max(12.0, 6 * noise)).any(axis=0).astype(np.uint8)
    moving = cv2.dilate(moving, np.ones((9, 9), np.uint8))
    return med, moving == 0


def register(ref, img, mask, iters=2):
    """Euclidean transform (shift + rotation) that lines img up with ref over
    the static pixels. Returns (2x3 warp mapping ref pixels to img pixels, n
    pixels used) or (None, 0). Pixels that still differ after a first fit
    (something moved between the two moments) are dropped and it is redone."""
    import cv2
    import numpy as np
    H, W = ref.shape
    border = np.zeros_like(mask)
    border[12:H - 12, 12:W - 12] = True
    m = mask & border
    # (no extra blur: on a smooth floor it flattens the match so much that ECC
    # stops ~15% short of the true shift; ECC's own 5 px smoothing is enough)
    a, b = ref, img
    win = cv2.createHanningWindow((W, H), cv2.CV_32F)
    (dx, dy), _ = cv2.phaseCorrelate(cv2.GaussianBlur(a, (0, 0), 2), cv2.GaussianBlur(b, (0, 0), 2), win)
    if math.hypot(dx, dy) > 0.25 * min(W, H):
        dx = dy = 0.0                                  # a nonsense rough guess: start from no shift
    warp = np.array([[1, 0, dx], [0, 1, dy]], np.float32)
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-6)
    for it in range(iters):
        if m.sum() < 2000:
            return None, int(m.sum())
        try:
            _, warp = cv2.findTransformECC(a, b, warp, cv2.MOTION_EUCLIDEAN, crit, m.astype(np.uint8), 5)
        except cv2.error:
            return None, int(m.sum())
        if it + 1 < iters:
            back = cv2.warpAffine(b, warp, (W, H), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
            r = np.abs(back - a)
            thr = max(8.0, 4 * 1.4826 * float(np.median(r[m])))
            m = m & (r < thr)
    return warp, int(m.sum())


def warp_motion(warp, W, H):
    """(shift of the image centre in px, rotation deg, largest corner shift px)."""
    import numpy as np
    pts = np.array([[W / 2, H / 2], [0, 0], [W, 0], [0, H], [W, H]], float)
    moved = pts @ warp[:, :2].T.astype(float) + warp[:, 2].astype(float)
    d = np.hypot(*(moved - pts).T)
    rot = math.degrees(math.atan2(float(warp[1, 0]), float(warp[0, 0])))
    return float(d[0]), rot, float(d[1:].max())


def camera_still_and_light_checks(rec, window_s=2.0, n_between=3, light_every_s=2.0, max_shift_px=0.5,
                                  max_rot_deg=0.1, max_light_pct=10.0, px_per_cm=3.4, video=None):
    """Checks 2 and 3: did the camera move (first ~2 s against the last ~2 s
    and a few moments between), and did the light change (static parts of
    the floor, sampled every light_every_s)."""
    import cv2
    import numpy as np
    name = os.path.basename(os.path.normpath(rec))
    cap = cv2.VideoCapture(video or os.path.join(rec, "video.avi"))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tw, _ = read_frames_csv(rec)
    if n <= 0:
        n = len(tw)
    fps = nominal_fps(rec) or 25.0
    if len(tw) > 2 and tw[-1] > tw[0]:
        fps = (len(tw) - 1) / (tw[-1] - tw[0])
    k = max(3, int(round(window_s * fps)))
    out = []
    if n < 2 * k:
        cap.release()
        return [Result("camera_moved", "Camera didn't move", name, "SKIP", float("nan"), "px", "", "video too short")]
    starts = [0] + [int(round((n - k) * (i + 1) / (n_between + 1))) for i in range(n_between)] + [n - k]
    views = []
    for s0 in starts:
        stack = read_window(cap, s0, k)
        if stack is None or len(stack) < 3:
            continue
        views.append((s0, *static_view(stack)))
    if len(views) < 2:
        cap.release()
        return [Result("camera_moved", "Camera didn't move", name, "SKIP", float("nan"), "px", "", "couldn't read video")]
    s_ref, ref, mref = views[0]
    H, W = ref.shape
    worst_shift, worst_rot, worst_corner, notes, failed = 0.0, 0.0, 0.0, [], []
    for s0, img, m in views[1:]:
        warp, npx = register(ref, img, mref & m)
        t_min = (s0 - s_ref) / fps / 60
        if warp is None:
            failed.append(f"{t_min:.1f} min")
            continue
        sh, rot, cor = warp_motion(warp, W, H)
        notes.append(f"{t_min:.1f} min: {sh:.2f} px, {rot:+.3f} deg")
        if sh > worst_shift:
            worst_shift = sh
        if abs(rot) > abs(worst_rot):
            worst_rot = rot
        worst_corner = max(worst_corner, cor)
    if failed and not notes:
        st_ = "FAIL"
    else:
        st_ = worst(grade(worst_shift, max_shift_px), grade(abs(worst_rot), max_rot_deg), "CHECK" if failed else "PASS")
    out.append(Result("camera_moved", "Camera didn't move", name, st_, worst_shift, "px",
                      f"shift < {max_shift_px:g} px (~{max_shift_px / px_per_cm:.2f} cm) and turn < {max_rot_deg:g} deg",
                      f"largest shift {worst_shift:.2f} px (~{worst_shift / px_per_cm:.2f} cm at the floor), largest turn "
                      f"{abs(worst_rot):.3f} deg (image corners moved up to {worst_corner:.2f} px), comparing the first "
                      f"{window_s:g} s with " + "; ".join(notes)
                      + (f"; could NOT line up the view at {', '.join(failed)} (bumped a lot, or blocked?)" if failed else "")))
    # lighting: brightness of the floor's static parts relative to the first window
    good = mref & (ref > 20)
    ratios, times = [], []
    step = max(1, int(round(light_every_s * fps)))
    if n // step > 400:
        step = n // 400
    for f in range(0, n, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, f)
        ok, img = cap.read()
        if not ok:
            break
        g = _gray(img)
        ratios.append(float(np.median(g[good] / ref[good])))
        times.append(f / fps)
    cap.release()
    if ratios:
        dev = [abs(r - 1) * 100 for r in ratios]
        i = int(np.argmax(dev))
        lo, hi = min(ratios), max(ratios)
        out.append(Result("lighting", "Lighting steady", name, grade(dev[i], max_light_pct), dev[i], "%",
                          f"brightness change < {max_light_pct:g}%",
                          f"floor brightness (start {float(np.median(ref[good])):.0f} of 255) ranged "
                          f"{(lo - 1) * 100:+.1f}% to {(hi - 1) * 100:+.1f}% of the start over {len(ratios)} samples; "
                          f"biggest change {dev[i]:.1f}% at {times[i] / 60:.1f} min"))
    return out


def flash_length_checks(rec, sync_video_times, sync_ms=300.0, max_blink_ms=220.0, frame_times=None, video=None,
                        edges_out=None):
    """Check 5: how long each light flash lasted in the video. Uses
    tools/blink/blink.py's flash finder (stages 1-2), which times each
    switch-on and switch-off to a fraction of the exposure. Sync flashes
    are the ones at a logged sync flash (video time); every other short
    flash is a blink flash (one 100 ms flash; firmware allows ~215 ms).
    edges_out (a dict) gets {index in sync_video_times: (video time on, video time off)}
    for each sync flash measured, for the clock-drift check."""
    import sys
    import numpy as np
    name = os.path.basename(os.path.normpath(rec))
    here = os.path.dirname(os.path.abspath(__file__))
    bdir = os.path.join(here, "blink")
    if bdir not in sys.path:
        sys.path.insert(0, bdir)
    import blink
    from dataclasses import replace
    p = blink.Params()
    p = replace(p, exposure_s=blink.exposure_from_meta(rec)[0])
    ft = np.array(frame_times if frame_times is not None else blink.read_frame_times(rec), float)
    segs, _ = blink.find_segments(video or os.path.join(rec, "video.avi"), blink.repair_late_frames(ft), p)
    lit = [s for s in segs if s.kind in ("pulse", "sync", "long")]
    dt = float(np.median(np.diff(ft))) if len(ft) > 2 else 0.04
    tol = 15.0 + max(0.0, dt - p.exposure_s) * 1000          # each end is known to about half the dark gap
    out = []
    sync_lens = []
    for i, tv in enumerate(sync_video_times):
        near = [s for s in lit if abs(s.a - tv) <= 0.15 and s.b - s.a > 0.15]
        if near:
            sync_lens.append(1000 * st.median(s.b - s.a for s in near))
            if edges_out is not None:
                edges_out[i] = (st.median(s.a for s in near), st.median(s.b for s in near))
    if sync_lens:
        dev = max(abs(x - sync_ms) for x in sync_lens)
        out.append(Result("sync_flash_length", "Sync flash length in the video", name, grade(dev, tol), dev, "ms",
                          f"within {tol:.0f} ms of {sync_ms:.0f} ms (timing resolution at this frame rate and exposure)",
                          f"{len(sync_lens)} of {len(sync_video_times)} sync flashes measured: "
                          f"{min(sync_lens):.0f} to {max(sync_lens):.0f} ms (median {st.median(sync_lens):.0f})"))
    else:
        out.append(Result("sync_flash_length", "Sync flash length in the video", name, "SKIP", float("nan"), "ms", "",
                          "no sync flash found to measure"))
    blinks = [1000 * (s.b - s.a) for s in lit
              if not any(abs(s.a - tv) <= 0.6 for tv in sync_video_times) and s.kind in ("pulse", "sync")]
    if blinks:
        mx = max(blinks)
        over = sum(1 for x in blinks if x >= max_blink_ms)
        out.append(Result("blink_flash_length", "Blink flash length in the video", name,
                          "PASS" if over == 0 else ("CHECK" if over <= max(1, 0.01 * len(blinks)) else "FAIL"), mx,
                          "ms", f"every blink flash < {max_blink_ms:.0f} ms",
                          f"{len(blinks)} flashes (each LED counted): median {st.median(blinks):.0f} ms, longest "
                          f"{mx:.0f} ms; {over} at or over {max_blink_ms:.0f} ms"))
    else:
        out.append(Result("blink_flash_length", "Blink flash length in the video", name, "SKIP", float("nan"), "ms",
                          "", "no blink flashes found (blinking off?)"))
    return out


# ---------------------------------------------------------------- home dock check tile (hardware/dock)
# Each home dock carries a printed black-and-white marker on top, fixed to the plywood: a spot the overhead camera
# sees in every frame that never moves. If its position in the image drifts, the camera (or the board) was bumped:
# every camera fix after that is off by the same amount. Dock v5 (hardware/dock/build_dock_v5.py, 2026-10-25): a
# strip of 2 x 6 squares of 15 mm (3.0 cm along the dock, 9.0 cm across it) on the base at the bottom of the U. Only
# ECC translation tracking uses it (no chessboard finder, no pose), so a rectangle works; the patch around it is a
# rectangle too, so the docked robot beside it stays out of the patch. (Docks v1-v4: a 4 cm square tile.) Dock v6
# (2026-10-28): the same pattern, size and place, now PRINTED (docs/hardware/checker-pieces-letter.pdf, piece E, on stiff
# card in a 0.6 mm pocket; tools/pi/checker_pieces.py): its top 0.05 mm lower than the 3D-printed strip's, so the
# numbers below stand. Tested on synthetic pictures: a 0.5 px camera bump reads 0.45-0.51 px.

DOCK_TILE_CENTER_CM = 1.84 + 0.4  # marker centre behind the bump faces (build_dock_v5 STRIP_CX_MM 18.4 behind the
                                  # U's inner face, + the bumps' 4 mm)
DOCK_TILE_SIZE_CM = (3.0, 9.0)  # along the dock's axis, across it (build_dock_v5 STRIP_DEP_MM, STRIP_LEN_MM)
DOCK_TILE_HEIGHT_CM = 2.2       # on top of the dock's 22 mm walls (seen slightly further out than its floor point)


def dock_tile_floor_points(map_path):
    """[(dock name, x_cm, y_cm)]: each home dock's check-tile centre on the floor, from a fleet / locked site
    map (home_docks)."""
    with open(map_path) as f:
        m = json.load(f)
    out = []
    for d in m.get("home_docks", []) or []:
        h = math.radians(float(d["heading"]))
        # the dock pose (docks v4-v5, backed into): the middle of the bump faces, heading = the way a docked robot
        # faces, OUT of the dock; the marker is on the base behind the bumps, so against the heading
        out.append((d.get("name", "dock"), float(d["x"]) - DOCK_TILE_CENTER_CM * math.cos(h),
                    float(d["y"]) - DOCK_TILE_CENTER_CM * math.sin(h)))
    return out


def dock_tile_corners(map_path):
    """[(dock name, [(x_cm, y_cm) x 4])]: the marker's four corners on the floor (map frame), for the patch size."""
    with open(map_path) as f:
        m = json.load(f)
    out = []
    for (name, cx, cy), d in zip(dock_tile_floor_points(map_path), m.get("home_docks", []) or []):
        h = math.radians(float(d["heading"]))
        ax, ay, bx, by = math.cos(h), math.sin(h), -math.sin(h), math.cos(h)
        a, b = DOCK_TILE_SIZE_CM[0] / 2, DOCK_TILE_SIZE_CM[1] / 2
        out.append((name, [(cx + sa * a * ax + sb * b * bx, cy + sa * a * ay + sb * b * by)
                           for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))]))
    return out


def floor_to_px(H, x, y):
    w = H[2][0] * x + H[2][1] * y + H[2][2]
    return ((H[0][0] * x + H[0][1] * y + H[0][2]) / w, (H[1][0] * x + H[1][1] * y + H[1][2]) / w)


class PatternWatch:
    """Tracks a fixed image patch (the dock's check tile) frame after frame: its shift (px) from where it was
    in the first frame, by ECC image alignment of a patch around it (translation only, sub-pixel: about
    0.01 px on a clean synthetic tile), started from the last shift found. Usable live (tools/live: feed it
    every n-th grey frame) and on a recording (dock_tile_checks)."""

    def __init__(self, center_px, half_px=12, margin_px=6, alarm_px=0.75, confirm=3):
        """half_px: half the patch's side (a square), or (half width, half height) for a rectangle."""
        self.cx, self.cy = float(center_px[0]), float(center_px[1])
        hw, hh = (half_px, half_px) if isinstance(half_px, (int, float)) else half_px
        self.hw, self.hh, self.m = int(hw), int(hh), int(margin_px)
        self.alarm_px, self.confirm = alarm_px, confirm
        self.ref = None
        self.last = (0.0, 0.0)
        self.high = 0
        self.history = []          # (t, dx, dy, match 0..1)

    def _patch(self, gray, extra):
        import numpy as np
        hw, hh = self.hw + extra, self.hh + extra
        x0, y0 = int(round(self.cx)) - hw, int(round(self.cy)) - hh
        H, W = gray.shape[:2]
        if x0 < 0 or y0 < 0 or x0 + 2 * hw > W or y0 + 2 * hh > H:
            return None
        return np.ascontiguousarray(gray[y0:y0 + 2 * hh, x0:x0 + 2 * hw], dtype=np.float32)

    def update(self, gray, t=0.0):
        """One frame (grey, float or uint8). Returns (dx, dy, match, alarm) or None if out of view. match is
        ECC's correlation (1 = the same picture); below 0.8 the tile was covered or moved too far to follow."""
        import cv2
        import numpy as np
        if self.ref is None:
            self.ref = self._patch(gray, 0)
            if self.ref is None:
                return None
            self.history.append((t, 0.0, 0.0, 1.0))
            return 0.0, 0.0, 1.0, False
        cur = self._patch(gray, self.m)
        if cur is None:
            return None
        warp = np.array([[1, 0, self.m + self.last[0]], [0, 1, self.m + self.last[1]]], np.float32)
        try:
            cc, warp = cv2.findTransformECC(self.ref, cur, warp, cv2.MOTION_TRANSLATION,
                                            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-5), None, 3)
        except cv2.error:
            self.history.append((t, float("nan"), float("nan"), 0.0))
            return float("nan"), float("nan"), 0.0, False
        dx, dy = float(warp[0, 2]) - self.m, float(warp[1, 2]) - self.m
        if cc >= 0.8:
            self.last = (dx, dy)
        self.history.append((t, dx, dy, float(cc)))
        moved = cc >= 0.8 and math.hypot(dx, dy) > self.alarm_px
        self.high = self.high + 1 if moved else 0
        return dx, dy, float(cc), self.high >= self.confirm


def dock_tile_checks(rec, map_path, calib_path, every_s=2.0, max_shift_px=0.5, px_per_cm=3.4, video=None):
    """The check tile on every home dock (map home_docks) must stay where it was at the start of the recording:
    its largest shift in px. A hand or a robot over the tile spoils a frame's match (low response): such frames
    are left out."""
    import cv2
    import numpy as np
    name = os.path.basename(os.path.normpath(rec))
    try:
        tiles = dock_tile_floor_points(map_path)
        with open(calib_path) as f:
            cal = json.load(f)
        H = None
        for k in ("H_floor_to_px", "homography_floor_cm_to_px", "H_floor_to_image", "H", "homography", "matrix",
                  "floor_to_image"):
            if cal.get(k) is not None:
                H = cal[k]
                break
    except (OSError, ValueError, KeyError) as e:
        return [Result("dock_tile", "Dock check tile didn't move", name, "SKIP", float("nan"), "px", "", str(e))]
    if not tiles or H is None:
        return [Result("dock_tile", "Dock check tile didn't move", name, "SKIP", float("nan"), "px", "",
                       "no home_docks in the map" if not tiles else "no floor-to-pixel homography in the calibration")]
    cap = cv2.VideoCapture(video or os.path.join(rec, "video.avi"))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = nominal_fps(rec) or 25.0
    step = max(1, int(round(every_s * fps)))
    watches = []
    corners = dict(dock_tile_corners(map_path))
    for dn, x, y in tiles:
        u, v = floor_to_px(H, x, y)
        # the marker's own outline in the picture plus 0.5 cm: a rectangle, so the docked robot stays outside it
        pc = [floor_to_px(H, a, b) for a, b in corners.get(dn, [(x, y)])]
        pad = 0.5 * px_per_cm
        hw = max(6, int(round(max(abs(q[0] - u) for q in pc) + pad)))
        hh = max(6, int(round(max(abs(q[1] - v) for q in pc) + pad)))
        watches.append((dn, PatternWatch((u, v), half_px=(hw, hh), margin_px=max(4, int(round(1.5 * px_per_cm))))))
    f = 0
    while n <= 0 or f < n:
        cap.set(cv2.CAP_PROP_POS_FRAMES, f)
        ok, img = cap.read()
        if not ok:
            break
        g = _gray(img)
        for _, w in watches:
            w.update(g, f / fps)
        f += step
    cap.release()
    out = []
    for dn, w in watches:
        good = [(t, dx, dy) for t, dx, dy, r in w.history[1:] if r >= 0.8]
        if not w.history:
            out.append(Result("dock_tile", "Dock check tile didn't move", f"{name} dock {dn}", "SKIP", float("nan"),
                              "px", "", "the tile is outside the picture (check the map's home dock and the calibration)"))
            continue
        if not good:
            out.append(Result("dock_tile", "Dock check tile didn't move", f"{name} dock {dn}", "CHECK", float("nan"),
                              "px", "", "no frame matched the first one (covered, or bumped a lot?)"))
            continue
        d = [math.hypot(dx, dy) for _, dx, dy in good]
        i = int(np.argmax(d))
        out.append(Result("dock_tile", "Dock check tile didn't move", f"{name} dock {dn}", grade(d[i], max_shift_px),
                          d[i], "px", f"shift < {max_shift_px:g} px (~{max_shift_px / px_per_cm:.2f} cm)",
                          f"largest shift {d[i]:.2f} px (~{d[i] / px_per_cm:.2f} cm) at {good[i][0] / 60:.1f} min, "
                          f"{len(good)} of {len(w.history) - 1} sampled frames matched; a shift means the camera (or "
                          "the dock) moved: camera fixes after it are off by about as much"))
    return out
