#!/usr/bin/env python3
"""Check that a recording session is complete and lines up, right after you
record it.

    python tools/check_session.py logs/20261018-101500
    python tools/check_session.py logs/20261018-101500 --video recordings/20261018-101455_E1

Reports, per robot and run:
  - full sensor log (RAW): rows received vs sent, rate, longest gap
  - battery at the start and end, and how the run ended
  - clock: how robot time converts to laptop time, round-trip times, drift
  - the robot's still-and-level IMU reading (checks the units)
With --video (a camrec.py recording folder; needs opencv-python and numpy):
  - camera frame rate and dropped frames
  - the light flashes found in the video, matched to the robot's own flash
    records, and the time offset between video and robot clocks

The laptop and Pi clocks are set over the internet and usually agree within
tens of milliseconds; the flash offset measures that difference plus the
camera's own delay, which is what latency work needs.

Then the automatic checks (tools/session_checks.py), each printed as one line
PASS / CHECK / FAIL (SKIP = no data, INFO = reported only) with its number and
pass rule, and all written to <session>/session_checks.csv
(docs/data-formats.md lists every check and rule):
  robot logs: WiFi loss and loss bursts; robot clock fit per boot; floor tilt
    and 1 g at rest; wheel/speed noise at rest against the fusion's 'still'
    test; gyro bias change at rest (warm-up); lifted rows; distance-sensor
    frame rate and reading age
  video: clock drift against the laptop over the session (writes
    <recording>/video_clock.json); dropped frames, frame rate, delivery
    jitter; exposure locked; camera settings start vs end; camera moved;
    lighting; sync and blink flash lengths (a second pass over the video:
    --quick skips it)
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import session_checks as sc  # noqa: E402


def rows(folder, name):
    out = []
    for path in sorted(glob.glob(os.path.join(folder, name + "*.csv"))):
        base = os.path.basename(path)[:-4]
        if base != name and not (base.startswith(name + "_") and base[len(name) + 1:].isdigit()):
            continue
        with open(path, newline="") as f:
            out += list(csv.DictReader(f))
    return out


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


# ------------------------------------------------------------------ blink codes
BLINK_NAMES = {"Left": 1, "Center": 2, "Right": 3}    # firmware/home_lab/blink.h codeFor(): exact, case-sensitive
BLINK_OTHER = 4                                     # any other name
BLINK_DEFAULT_NAMES = {1: "Left", 2: "Center", 3: "Right", 4: "other"}   # = tools/blink/blink.py DEFAULT_NAMES


def blink_code_for(name, blink_code):
    """The code a robot blinks, exactly as the firmware's codeFor(): PARAM blink_code
    1..4 is used as given; anything else (0 = default) takes it from the name."""
    v = fnum(blink_code, 0.0)
    c = int(v) if v == v else 0           # (int)P("blink_code") in the firmware
    if 1 <= c <= 4:
        return c
    return BLINK_NAMES.get(name, BLINK_OTHER)


def session_blink_codes(folder):
    """What each robot of a console session blinks, from its runs' PARAM records.
    Returns (codes, notes): codes = {robot: {code: [run ids]}} (runs with
    blink_codes = 0 don't blink and are left out); notes = list of warnings:
    a non-zero blink_code, a name that isn't exactly Left / Center / Right
    (case matters: "left" blinks the 'other' code), one robot blinking
    different codes in different runs, two robots blinking the same code."""
    params = defaultdict(dict)
    for r in rows(folder, "param"):
        params[(r["robot"], r["run"])][r["name"]] = r["value"]
    codes = defaultdict(lambda: defaultdict(list))
    nonzero = defaultdict(list)
    notes = []
    for (robot, run), pr in sorted(params.items()):
        if "blink_codes" in pr and fnum(pr["blink_codes"], 1) <= 0:
            continue
        bc = pr.get("blink_code", "0")
        c = blink_code_for(robot, bc)
        if fnum(bc, 0) != 0:
            nonzero[robot].append((run, f"{fnum(bc):g}", c))
        codes[robot][c].append(run)
    for robot, lst in sorted(nonzero.items()):
        vals = sorted({v for _, v, _ in lst})
        notes.append(f"{robot}: PARAM blink_code = {'/'.join(vals)} (not 0) in {len(lst)} run(s) "
                     f"(e.g. {lst[0][0]}): it blinks code {'/'.join(sorted({str(c) for _, _, c in lst}))}"
                     + (" whatever its name" if all(1 <= int(float(v)) <= 4 for v in vals)
                        else " (a value outside 1..4 falls back to the name)"))
    for robot in sorted(codes):
        if robot not in BLINK_NAMES:
            near = next((n for n in BLINK_NAMES if n.lower() == robot.strip().lower()), None)
            notes.append(f"robot name {robot!r} is not one of Left / Center / Right"
                         + (f" (the firmware compares exactly: {robot!r} is not {near!r})" if near else "")
                         + f": unless blink_code is set it blinks the 'other' code {BLINK_OTHER}")
        if len(codes[robot]) > 1:
            notes.append(f"{robot} blinks different codes in different runs: "
                         + ", ".join(f"code {c} in {len(r)} run(s)" for c, r in sorted(codes[robot].items())))
    by_code = defaultdict(set)
    for robot, cs in codes.items():
        for c in cs:
            by_code[c].add(robot)
    for c, robots in sorted(by_code.items()):
        if len(robots) > 1:
            notes.append(f"{' and '.join(sorted(robots))} both blink code {c}: the camera can't tell them apart "
                         "by their lights (set PARAM blink_code so each has its own, or rename)")
    return {r: dict(cs) for r, cs in codes.items()}, notes


def blink_names(codes):
    """{code: name} for tools/blink/blink.py from session_blink_codes(): each code
    named after the robot that blinks it; a code two robots share gets both names
    joined by '/' (no robot matches it: the fusion treats it as an unknown name);
    unused codes keep their default name unless a robot already has it."""
    users = defaultdict(set)
    for robot, cs in codes.items():
        for c in cs:
            users[c].add(robot)
    names = {c: "/".join(sorted(rb)) for c, rb in users.items()}
    taken = set(names.values())
    for c in range(1, BLINK_OTHER + 1):
        if c not in names:
            d = BLINK_DEFAULT_NAMES[c]
            names[c] = d if d not in taken else f"code{c}"
    return dict(sorted(names.items()))


# ------------------------------------------------------------------ clock
def clock_fits(folder):
    """(robot, boot) -> (a, b, info): laptop_time = a + b * robot_ms / 1000, from
    the fastest answer in each burst of clock checks. Bursts that were slow even
    at best (answered while the robot was busy) are left out."""
    by = defaultdict(list)
    for r in rows(folder, "clock_sync"):
        by[(r["robot"], r.get("boot", ""))].append(r)
    fits = {}
    for key, rs in by.items():
        rs.sort(key=lambda r: fnum(r["t_send"]))
        bursts, cur = [], []
        for r in rs:   # answers less than 1 s apart belong to one burst
            if cur and fnum(r["t_send"]) - fnum(cur[-1]["t_send"]) > 1.0:
                bursts.append(cur)
                cur = []
            cur.append(r)
        if cur:
            bursts.append(cur)
        best = [min(b, key=lambda r: fnum(r["rtt_ms"])) for b in bursts]
        med_best = st.median(fnum(r["rtt_ms"]) for r in best)
        kept = [r for r in best if fnum(r["rtt_ms"]) <= max(2 * med_best, med_best + 10)]
        xs = [fnum(r["robot_ms"]) / 1000 for r in kept]
        ys = [(fnum(r["t_send"]) + fnum(r["t_recv"])) / 2 for r in kept]
        rtts = [fnum(r["rtt_ms"]) for r in rs]
        if len(kept) >= 2 and max(xs) - min(xs) > 5:
            mx, my = st.mean(xs), st.mean(ys)
            b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
            a = my - b * mx
        else:
            b = 1.0
            a = st.median(y - x for x, y in zip(xs, ys))
        resid = [y - (a + b * x) for x, y in zip(xs, ys)]
        fits[key] = (a, b, dict(n=len(rs), bursts=len(best), used=len(kept), rtt_med=st.median(rtts),
                                rtt_max=max(rtts), drift_ppm=(1 / b - 1) * 1e6,   # + = robot clock runs fast vs laptop
                                resid_ms=max(abs(e) for e in resid) * 1000))
    return fits


def fit_for(fits, robot, run):
    """The clock fit for the boot a run belongs to (run ids start with the boot id)."""
    boot = run.split("-")[0] if run else ""
    if (robot, boot) in fits:
        return fits[(robot, boot)]
    same = [v for (r, bt), v in fits.items() if r == robot and bt == ""]   # older console: no boot column
    return same[0] if len(same) == 1 else None


def to_laptop(fit, ms):
    a, b, _ = fit
    return a + b * ms / 1000


# ------------------------------------------------------------------ video
def recording_fps(folder, frames=None):
    """Frame rate of a camrec.py recording: from frames.csv's times (the real rate), else the camera's
    setting in meta.json (session_checks.nominal_fps), else 25 (the T1 camera; only when the folder says
    nothing)."""
    if frames and len(frames) > 2 and frames[-1] > frames[0]:
        return (len(frames) - 1) / (frames[-1] - frames[0])
    try:
        v = sc.nominal_fps(folder)
    except (OSError, ValueError, TypeError, AttributeError):
        v = None
    return v or 25.0


def exposure_s(folder, dt_frame):
    """Exposure time of each frame, from the recorder's meta.json when it knows
    it (camrec.py writes exposure_ms when exposure is locked), else assume the
    exposure fills the whole frame interval (typical indoors)."""
    try:
        with open(os.path.join(folder, "meta.json")) as f:
            m = json.load(f)
        e = m.get("exposure_ms")
        if e:
            return min(float(e) / 1000.0, dt_frame)
    except (OSError, ValueError, TypeError):
        pass
    return dt_frame


def video_flashes(folder, sync_ms_guess=300):
    """Find light flashes: a jump in brightness in a small spot, followed by
    the opposite jump about sync_ms later. Returns candidate "on" frames as
    [(t_wall, frame, strength)]; the caller keeps the strongest one near each
    flash the robot logged, which rules out look-alikes elsewhere in the video."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("  (install opencv-python and numpy to check the video: pip install opencv-python numpy)")
        return None, None
    frames = []
    with open(os.path.join(folder, "frames.csv"), newline="") as f:
        for r in csv.DictReader(f):
            frames.append(fnum(r["t_wall"]))
    cap = cv2.VideoCapture(os.path.join(folder, "video.avi"))
    fps = recording_fps(folder, frames)
    dt_frame = 1.0 / fps
    expo = exposure_s(folder, dt_frame)
    span = sync_ms_guess / 1000 * fps                  # flash length in frames
    smin, smax = max(2, int(span) - 1), int(math.ceil(span)) + 1
    # A light flash: pixels that brighten at frame j, STAY bright for about the
    # flash length, then darken. Moving robots and hands brighten and darken
    # pixels too, but almost never in that pattern.
    buf, scores = [], []   # scores[j] = (strength, x, y) for "a flash turned on at frame j + 1" (buf[0] is frame j)
    k = 4                  # strongest pixels averaged; a light seen from 2 m covers only a few pixels

    def score_at(b):       # b[0] is the frame before the candidate "on" frame
        before = b[0]
        on = np.min(np.stack(b[1:1 + smin]), axis=0) - before          # stayed brighter than before
        offs = [b[1 + s - 1] - b[1 + s] for s in range(smin, smax + 1) if 1 + s < len(b)]
        off = np.max(np.stack(offs), axis=0)                            # then went dark again
        p = np.minimum(on, off)
        # A flash that switches on or off partway through a frame's exposure
        # splits its brightness change over two frames, halving each jump.
        # Also score it with one in-between frame skipped at each edge.
        on2 = np.min(np.stack(b[2:2 + smin]), axis=0) - before
        offs2 = [b[1 + s - 1] - b[1 + s + 1] for s in range(smin, smax + 1) if 2 + s < len(b)]
        off2 = np.max(np.stack(offs2), axis=0)
        p = np.maximum(p, np.minimum(on2, off2)).ravel()
        idx = np.argpartition(p, -k)[-k:]
        ys, xs = np.unravel_index(idx, before.shape)
        # how much of the first bright frame was lit: 1 = the whole exposure,
        # 0.5 = the light came on halfway through it
        rise = (b[2].ravel()[idx] - before.ravel()[idx]).astype(float)
        part = (b[1].ravel()[idx] - before.ravel()[idx]) / np.maximum(rise, 1.0)
        lit = float(np.clip(np.median(part), 0.0, 1.0))
        return float(np.mean(p[idx])), float(np.median(xs)), float(np.median(ys)), lit

    while True:
        ok, img = cap.read()
        if not ok:
            break
        buf.append(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.int16))
        if len(buf) == smax + 3:
            scores.append(score_at(buf))
            buf.pop(0)
    cap.release()
    if len(scores) < 3:
        return [], frames
    # A flash must stand out from what is happening around it, not from the
    # whole video: a fixed threshold set from the whole video sat above flashes
    # whose brightness change was split over two frames whenever robots moved
    # a lot elsewhere in the recording. Keep each local peak that is at least
    # 1.5x stronger than the strongest score in the second around it (with the
    # flash itself left out). Callers then take the strongest peak near each
    # flash the robot logged.
    v = np.array([x[0] for x in scores])
    w = int(round(fps))                     # one second of frames
    guard = smax + 2                        # the flash's own frames
    found = []
    for j in range(len(v)):
        if v[j] < 40 or j + 1 >= len(frames):
            continue
        lo, hi = max(0, j - guard), min(len(v), j + guard + 1)
        if v[j] < v[lo:hi].max():
            continue                        # not the peak of this flash
        around = np.concatenate([v[max(0, j - w):lo], v[hi:j + w + 1]])
        bg = float(around.max()) if around.size else 0.0
        if v[j] >= 1.5 * bg:
            # When did the light come on, on the video clock, in the same sense
            # as any other frame's time? A frame stamped t was exposed over
            # [t - L - E, t - L]; positions in it belong to its exposure. The
            # light lit a fraction `lit` of the first lit frame's exposure, so it
            # came on lit*E before that frame's exposure ended. A fully lit first
            # frame means it came on somewhere in the dark gap before that
            # exposure: take the middle of the gap.
            lit = scores[j][3]
            t_on = frames[j + 1] - lit * expo
            if lit >= 0.98:
                t_on -= max(0.0, dt_frame - expo) / 2
            found.append((t_on, j + 1, float(v[j]), bg))
    if os.environ.get("FLASH_DEBUG"):
        print("flash peaks kept:", len(found))
    return found, frames


def coarse_video_offset(found, logged, max_s=300.0, bin_s=0.02):
    """Rough (video clock - laptop clock) from the whole flash sequence: the
    time shift that lines up the most logged flashes with video flashes.
    Works even when the Pi's and laptop's clocks are minutes apart, where
    matching each flash in a fixed window would lock onto look-alikes.
    Returns (offset_s, n_flashes_agreeing) or (None, 0)."""
    from collections import Counter
    votes = Counter()
    for t in logged:
        mine = set()
        for f in found:
            d = f[0] - t
            if abs(d) <= max_s:
                b = round(d / bin_s)
                mine.update((b - 1, b, b + 1))      # tolerate falling on a bin edge
        votes.update(mine)
    if not votes:
        return None, 0
    b, n = votes.most_common(1)[0]
    if n < 2:
        return None, n
    ds = [f[0] - t for t in logged for f in found if abs(f[0] - t - b * bin_s) <= 1.5 * bin_s]
    return st.median(ds), n


def match_flashes(found, logged, frame_s=1 / 30):
    """Match each logged flash time (laptop clock) to a video flash.
    Returns (coarse_offset_s or None, [candidate or None per logged flash]).
    A match must sit within 3 frames of where the flash sequence as a whole
    says it should be."""
    coarse, n = coarse_video_offset(found, logged)
    if coarse is None:
        return None, [None] * len(logged)
    out = []
    for t in logged:
        c = [f for f in found if abs(f[0] - (t + coarse)) <= 3 * frame_s]
        out.append(max(c, key=lambda f: f[2]) if c else None)
    return coarse, out


# ------------------------------------------------------------------ report
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="console session folder, e.g. logs/20261018-101500")
    ap.add_argument("--video", nargs="*", default=[], help="camrec.py recording folder(s) from the same session")
    ap.add_argument("--quick", action="store_true", help="skip the flash-length check (reads the whole video "
                                                          "a second time)")
    ap.add_argument("--px-per-cm", type=float, default=3.4, help="camera pixels per floor cm, for the camera-move "
                                                                 "check's cm figure (default 3.4)")
    args = ap.parse_args(argv)
    S = args.session
    results = []

    info = rows(S, "runinfo")
    if not info:
        print("No RUNINFO records here: was the robot running firmware 2026-10-17a or later?")
    ends = {(r["robot"], r["run"]): r for r in rows(S, "runend")}
    looks = {r["run"]: r for r in rows(S, "calib_look")}     # calibration runs, firmware 2026-10-18c+
    calib_outcomes = []
    raw = defaultdict(list)
    for r in rows(S, "raw"):
        raw[(r["robot"], r["run"])].append(r)
    params = defaultdict(dict)
    for r in rows(S, "param"):
        params[(r["robot"], r["run"])][r["name"]] = r["value"]

    print(f"\nSESSION {os.path.basename(os.path.normpath(S))}")
    try:
        with open(os.path.join(S, "session.json")) as f:
            meta = json.load(f)
        print(f"  laptop {meta.get('laptop')}, repo commit {meta.get('repo_commit')}, started {meta.get('start')}")
    except (OSError, ValueError):
        pass

    print("\nRUNS")
    hdr = ["robot", "run", "experiment", "fw", "result", "secs", "battery", "raw got/sent", "loss", "rate/s", "max gap ms", "az still"]
    table = []
    problems = []
    for r in info:
        key = (r["robot"], r["run"])
        e = ends.get(key, {})
        rr = raw.get(key, [])
        rr = sorted(rr, key=lambda x: fnum(x["ms"]))
        ms = [fnum(x["ms"]) for x in rr]
        sent = int(fnum(e.get("raw_rows_sent"), 0)) if e else 0
        got = len(rr)
        loss = 100 * (1 - got / sent) if sent else float("nan")
        rate = (len(ms) - 1) / ((ms[-1] - ms[0]) / 1000) if len(ms) > 2 and ms[-1] > ms[0] else float("nan")
        gap = max((b - a for a, b in zip(ms, ms[1:])), default=float("nan"))
        rn = [fnum(x.get("raw_n")) for x in rr]
        if rr and not any(v != v for v in rn):
            # firmware 2026-10-18b+: raw_n numbers the rows, so lost rows and real pauses separate:
            # the longest gap between CONSECUTIVE rows is a pause on the robot, not WiFi loss
            lost_n = sum(max(0, int(b - a) - 1) for a, b in zip(rn, rn[1:]))
            gap = max((b - a for a, b, p, q in zip(ms, ms[1:], rn, rn[1:]) if q - p == 1), default=float("nan"))
            if lost_n:
                problems.append(f"{key}: {lost_n} sensor rows lost in transit (raw_n jumps)")
        still = [fnum(x["az"]) for x in rr if x.get("step") == "0"][:20]   # before the robot moves
        az = st.mean(still) if still else float("nan")
        secs = fnum(e.get("duration_ms")) / 1000 if e else float("nan")
        bat = f"{r['battery']}->{e.get('battery', '?')}%"
        res = e.get("result", "no end record")
        if r["experiment"] == "calib":
            oc = sc.calib_outcome(e.get("result"), looks.get(r["run"]))
            calib_outcomes.append(oc)
            if oc == sc.CALIB_LIMITED:
                res += " (limited: no room)"
        table.append([r["robot"], r["run"], r["experiment"], r["fw_version"], res,
                      f"{secs:.1f}", bat, f"{got}/{sent}", f"{loss:.1f}%", f"{rate:.0f}", f"{gap:.0f}", f"{az:.2f}"])
        if not e:
            problems.append(f"{key}: no RUNEND record (run still going, or its packet was lost)")
        if sent and loss > 2:
            problems.append(f"{key}: {loss:.1f}% of sensor rows lost; consider raw_hz 25")
        if gap == gap and gap > 300:   # library move()/rotate() block ~200 ms by design
            problems.append(f"{key}: a {gap:.0f} ms gap in the sensor log")
        results += sc.wifi_checks(key, rr, sent)
    if table:
        w = [max(len(h), *(len(str(t[i])) for t in table)) for i, h in enumerate(hdr)]
        print("  " + "  ".join(h.ljust(w[i]) for i, h in enumerate(hdr)))
        for t in table:
            print("  " + "  ".join(str(c).ljust(w[i]) for i, c in enumerate(t)))
        print("  az still: vertical acceleration while the robot sits still before moving; about 1.00 means the"
              "\n  accelerometer columns are in g.")
    if calib_outcomes:
        print("  calibration runs: " + ", ".join(f"{calib_outcomes.count(k)} {k}" for k in
                                                (sc.CALIB_DONE, sc.CALIB_LIMITED, sc.CALIB_FAILED))
              + " (limited: it skipped the spin or the roll for lack of room; not a failure)")

    print("\nCLOCKS (robot time -> laptop time)")
    fits = clock_fits(S)
    if not fits:
        print("  no clock checks recorded")
    for (robot, boot), (a, b, i) in sorted(fits.items()):
        print(f"  {robot} (boot {boot or '?'}): {i['n']} checks in {i['bursts']} bursts ({i['used']} used); "
              f"round trip median {i['rtt_med']:.1f} ms, "
              f"worst {i['rtt_max']:.1f} ms; robot clock {abs(i['drift_ppm']):.0f} ppm {'fast' if i['drift_ppm'] >= 0 else 'slow'}; fit within {i['resid_ms']:.1f} ms")
        if i["rtt_med"] > 50:
            problems.append(f"{robot}: slow WiFi round trips ({i['rtt_med']:.0f} ms median)")
    results += sc.clock_checks(fits)

    syncs = rows(S, "sync")
    sync_ms = [fnum(r["value"]) for r in rows(S, "param") if r["name"] == "sync_ms"]
    sync_len = st.median(sync_ms) if sync_ms else 300.0
    lag = [fnum(sy.get("ms_on_after")) - fnum(sy["ms_on"]) for sy in syncs if sy.get("ms_on_after") not in (None, "")]
    if lag:   # firmware 2026-10-18b+: time spent inside the two set_color() calls that switch the flash on
        print(f"\nSYNC  light switch-on calls took median {st.median(lag):.0f} ms, max {max(lag):.0f} ms "
              f"({len(lag)} flashes; ms_on_after - ms_on)")
    for sy in syncs:
        if fit_for(fits, sy["robot"], sy["run"]) is None:
            problems.append(f"{sy['robot']} {sy['run']}: no clock checks for this boot, flash can't be placed in time")
    coarse_by_video = {}
    if args.video:
        print("\nVIDEO")
    for vf in args.video:
        found, frames = video_flashes(vf, sync_len)
        if frames is None:
            continue
        name = os.path.basename(os.path.normpath(vf))
        fps = recording_fps(vf, frames)
        if len(frames) > 2:
            iv = [b - a for a, b in zip(frames, frames[1:])]
            slow = sum(1 for x in iv if x > 1.5 * st.median(iv))
            print(f"  {name}: {len(frames)} frames, {fps:.1f} fps, {slow} late frames (>1.5x normal spacing), "
                  f"longest gap {max(iv) * 1000:.0f} ms")
        placed = []
        for sy in syncs:
            fit = fit_for(fits, sy["robot"], sy["run"])
            if fit:
                placed.append((to_laptop(fit, fnum(sy["ms_on"])), sy))
        offs, pairs, in_video = [], [], []
        coarse, picks = match_flashes(found or [], [t for t, _ in placed], 1 / fps)
        coarse_by_video[vf] = coarse or 0.0
        if coarse is None and placed:
            print("    the flash sequence in the video doesn't line up with the robots' flashes: is this the "
                  "right recording, and are the lights in view?")
            problems.append(f"{name}: flashes could not be lined up with the robot logs")
        elif placed:
            print(f"  video clock - laptop clock, rough: {coarse * 1000:+.0f} ms")
        for (t, sy), near in zip(placed, picks):
            label = f"{sy['robot']} run {sy['run']} {sy['tag']} flash"
            tv = t + (coarse or 0.0)
            if not frames or not (frames[0] - 1 <= tv <= frames[-1] + 1):
                continue   # recorded outside this video; reported below if no video covers it
            in_video.append((tv, t, to_laptop(fit_for(fits, sy["robot"], sy["run"]), fnum(sy["ms_off"]))))
            crowd = [o for o, q in placed if q is not sy and abs(o - t) < 1.0]
            if crowd:
                print(f"    {label}: another flash within 1 s, can't tell them apart (run robots one at a time)")
                problems.append(f"{label}: overlapping flashes")
                continue
            if near:
                offs.append(near[0] - t)
                pairs.append((t, near[0] - t))
                print(f"    {label}: video frame {near[1]}, video minus robot time {1000 * (near[0] - t):+.0f} ms")
            elif coarse is not None:
                print(f"    {label}: not seen in the video (lights out of view, or too small at this camera height?)")
                problems.append(f"{label} not found in {name}")
        if len(offs) >= 2:
            print(f"  video vs robot clock: {1000 * st.mean(offs):+.0f} ms on average, spread "
                  f"{1000 * st.stdev(offs):.0f} ms (one frame is {1000 / fps:.0f} ms)")
            if st.stdev(offs) > 0.05:
                problems.append("flash offsets vary by more than 50 ms: check the Pi's and laptop's clocks")
        edges = {}
        if args.quick:
            results.append(sc.Result("flash_length", "Flash lengths in the video", name, "SKIP", float("nan"), "",
                                     "", "skipped (--quick)"))
        else:
            results += sc.flash_length_checks(vf, [x[0] for x in in_video], sync_len, frame_times=frames,
                                              edges_out=edges)
        # Clock drift from the sync flashes. Both edges of each flash (switch-on and
        # switch-off, from the flash-length pass) where measured: when the flash
        # isn't a whole number of frames long (300 ms at 25 fps = 7.5 frames) their
        # timing errors partly cancel. Else the switch-on times found above.
        if len(edges) >= 2:
            pairs2 = [(t_on, ((a - t_on) + (b - t_off)) / 2) for i, (tv, t_on, t_off) in enumerate(in_video)
                      if i in edges for a, b in [edges[i]]]
            results += sc.video_clock_check(name, pairs2, write_to=vf, how="both edges", level_pairs=pairs)
        else:
            results += sc.video_clock_check(name, pairs, write_to=vf)
        results += sc.frame_timing_checks(vf)
        results += sc.camera_settings_check(vf)
        results += sc.camera_still_and_light_checks(vf, px_per_cm=args.px_per_cm)
    if args.video:
        spans = []
        for vf in args.video:
            try:
                with open(os.path.join(vf, "frames.csv"), newline="") as f:
                    ts = [fnum(r["t_wall"]) for r in csv.DictReader(f)]
                if ts:
                    c = coarse_by_video.get(vf, 0.0)          # video clock -> laptop clock
                    spans.append((ts[0] - c - 1, ts[-1] - c + 1))
            except OSError:
                pass
        for sy in syncs:
            fit = fit_for(fits, sy["robot"], sy["run"])
            if fit:
                t = to_laptop(fit, fnum(sy["ms_on"]))
                if not any(a <= t <= b for a, b in spans):
                    problems.append(f"{sy['robot']} run {sy['run']} {sy['tag']} flash falls outside every video's "
                                    f"time range (recorder not running, or the Pi's clock is off?)")

    print("\nBLINK CODES (from each run's PARAM blink_code and the robot's name)")
    bcodes, bnotes = session_blink_codes(S)
    if not bcodes:
        print("  no PARAM records with blinking on")
    for robot, cs in sorted(bcodes.items()):
        print(f"  {robot}: " + ", ".join(f"code {c} ({len(r)} run(s), flash every {(c + 2) / 10:.1f} s)"
                                        for c, r in sorted(cs.items())))
    for n in bnotes:
        problems.append("blink: " + n)
    if bcodes:
        nm = blink_names(bcodes)
        if nm != BLINK_DEFAULT_NAMES:
            print("  not the default code -> name mapping: run tools/blink/blink.py with --session " + S)

    if not args.video:
        for cid, title in (("video_clock_drift", "Video clock drift vs laptop"), ("camera_moved", "Camera didn't move"),
                           ("lighting", "Lighting steady"), ("frame_drops", "Dropped camera frames"),
                           ("frame_jitter", "Frame delivery jitter"), ("flash_length", "Flash lengths in the video"),
                           ("exposure", "Exposure locked and known")):
            results.append(sc.Result(cid, title, "no recording", "SKIP", float("nan"), "", "",
                                     "no recording given (add --video <recording folder>)"))
    results += sc.tilt_checks(raw)
    results += sc.rest_noise_checks(raw)
    results += sc.gyro_bias_checks(raw)
    results += sc.lifted_checks(raw)
    results += sc.tof_checks(raw)
    for p in problems:
        results.append(sc.Result("note", "Other problem", "session", "CHECK", float("nan"), "", "", p))

    print("\nCHECKS (PASS = fine; CHECK = look at it before trusting the data; FAIL = this data has a problem;"
          "\n        SKIP = couldn't check; INFO = reported only)")
    for r in results:
        if r.check != "note":
            print("  " + r.line())
    print("\nOTHER PROBLEMS")
    if problems:
        for p in problems:
            print("  - " + p)
    else:
        print("  none")
    out = os.path.join(S, "session_checks.csv")
    try:
        sc.write_csv(out, results)
        counts = {k: sum(1 for r in results if r.status == k) for k in ("PASS", "CHECK", "FAIL", "SKIP", "INFO")}
        print(f"\n{', '.join(f'{v} {k}' for k, v in counts.items() if v)}; all results in {out}")
    except OSError as e:
        print(f"\ncould not write {out}: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
