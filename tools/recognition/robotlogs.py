"""The Alvik plug-in for outside data: reads a console session folder (tools/alvik_console.py,
tools/roam_session.py: logs/<date-time>[_roam]/) and returns one own-frame Stream per robot on
the recording's clock (t_wall), for tools/recognition/outside.py.

    data = read_alvik_logs("logs/20261020-070600_roam", recording="recordings/20261020-070555_roam1")
    data = outside.read_outside(folder, plugin="alvik", recording=...)

What it reads (docs/data-formats.md "Console session folder"); every file is optional except raw.csv,
and a missing or damaged file costs only what it carries (said in data.notes):

  raw*.csv          robot,t_recv,run,ms,... x_cm,y_cm,th_deg (library odometry, reset to 0 at the start of
                    straight and demo runs), yaw_deg (gyro), wl_deg, wr_deg, lifted
  runinfo*.csv      yaw_sign, odo_sign per run (0 = not checked: the heading's sign is then fitted)
  clock_sync.csv    robot clock -> laptop clock (fastest answer of each burst, as check_session.clock_fits)
  clock.csv         the robots' clock answers without the question time: used, like the RAW rows' t_recv,
                    as a lower envelope when clock_sync.csv is missing
  param.csv         gyro_scale, wheel_scale in use (firmware 2026-10-20b+): priors for the fit
  dock.csv, posts.csv, runend.csv, sync.csv: events (docked, post touch, run end, light flashes)

Clock: robot ms -> laptop (per robot and boot) -> t_wall (Pi): the laptop -> Pi step comes from
`laptop_to_wall` (seconds to add, or a LinearClock), else <recording>/video_clock.json
(tools/check_session.py: video t_wall = laptop + offset_s [+ drift]), else 0 with a note
(outside.estimate_time_offset can then find it from the motion).

Path: distance from the change of x_cm, y_cm between rows (signed by the library's own heading, so
reversing counts backwards; resets at run starts are recognised), heading from the gyro (yaw_deg x
yaw_sign, continuous over the whole boot; odometry th_deg when there is no yaw). Rows with lifted = 1
are not placed and start a new segment (the robot was carried: its position must be found again).

Alvik knowledge used here (and only here): heading sign columns, run resets, the gyro under-reporting
turns by ~7.5 % (docs/session-notes/2026-10-19.md: prior gyro scale 1.08), the T1 wheel scale ~0.979.
"""
import csv
import glob
import json
import math
import os
import statistics as stats

import numpy as np

try:
    from .outside import OutsideData, Plugin, Stream, LinearClock, register
except ImportError:
    from outside import OutsideData, Plugin, Stream, LinearClock, register

WHEEL_DIAM_CM = 3.4           # Alvik wheel (34 mm, datasheet): only when x_cm, y_cm are missing
RUN_RESET_CM = 2.0            # first row of a run this close to (0, 0): the library pose was reset
RESET_TURN_DEG = 3.0          # a heading jump this big across a run boundary is a reset, not a turn
NEW_SEGMENT_GAP_S = 30.0      # no rows for this long: it may have been moved by hand meanwhile
STILL_CM = 0.1                # less than this driven over STILL_WINDOW_S counts as still
STILL_WINDOW_S = 0.5
ALVIK_PRIORS = dict(g=1.08, g_sd=0.04, d=0.0, d_sd=0.4, s=0.979, s_sd=0.02)


def _f(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except (TypeError, ValueError):
        return default


def read_rows(folder, rtype):
    """All rows of <rtype>.csv and <rtype>_2.csv ... (a header change starts a new file). Damaged
    lines (a crash mid-write, NUL bytes) are skipped. Returns (rows, files, skipped)."""
    files = sorted(glob.glob(os.path.join(folder, rtype + ".csv")) + glob.glob(os.path.join(folder, rtype + "_*.csv")))
    files = [p for p in files if os.path.basename(p) == rtype + ".csv" or
             os.path.basename(p)[len(rtype) + 1:-4].isdigit()]
    rows, skipped = [], 0
    for p in files:
        try:
            with open(p, newline="", encoding="utf-8", errors="replace") as f:
                lines = (ln.replace("\0", "") for ln in f)
                rd = csv.reader(lines)
                head = next(rd, None)
                if not head:
                    continue
                head = [h.strip() for h in head]
                for r in rd:
                    if len(r) != len(head):
                        skipped += 1
                        continue
                    rows.append(dict(zip(head, r)))
        except OSError:
            skipped += 1
    return rows, files, skipped


def _boot(run):
    return (run or "").split("-")[0] if run else ""


# =============================================================================== clocks
def clock_fits(folder):
    """{(robot, boot): (a, b, info)}: laptop = a + b * robot_ms / 1000, from clock_sync.csv (the
    fastest answer of each burst; same method as tools/check_session.py clock_fits)."""
    rows, _, _ = read_rows(folder, "clock_sync")
    by = {}
    for r in rows:
        ts, tr, ms, rtt = _f(r.get("t_send")), _f(r.get("t_recv")), _f(r.get("robot_ms")), _f(r.get("rtt_ms"))
        if not (math.isfinite(ts) and math.isfinite(tr) and math.isfinite(ms)):
            continue
        if not math.isfinite(rtt):
            rtt = (tr - ts) * 1000
        by.setdefault((r.get("robot", ""), r.get("boot", "")), []).append((ts, tr, ms, rtt))
    fits = {}
    for key, rs in by.items():
        rs.sort()
        bursts, cur = [], []
        for r in rs:
            if cur and r[0] - cur[-1][0] > 1.0:
                bursts.append(cur)
                cur = []
            cur.append(r)
        if cur:
            bursts.append(cur)
        best = [min(b, key=lambda r: r[3]) for b in bursts]
        med = stats.median(r[3] for r in best)
        kept = [r for r in best if r[3] <= max(2 * med, med + 10)]
        xs = np.array([r[2] / 1000 for r in kept])
        ys = np.array([(r[0] + r[1]) / 2 for r in kept])
        if len(kept) >= 2 and xs.max() - xs.min() > 5:
            b, a = np.polyfit(xs, ys, 1)
        else:
            b, a = 1.0, float(np.median(ys - xs))
        fits[key] = (float(a), float(b), dict(n=len(rs), used=len(kept), source="clock_sync.csv",
                                              rtt_med_ms=float(med)))
    return fits


def envelope_fit(robot_ms, t_recv):
    """laptop = a + b * ms / 1000 from arrival times alone: a packet arrives AFTER it was sent, so
    the lowest (t_recv - ms/1000) in each 30 s stretch is the clock offset plus the shortest delay
    (a few ms). A line through those minima, or their minimum for a short log."""
    ms, tr = np.asarray(robot_ms, float), np.asarray(t_recv, float)
    ok = np.isfinite(ms) & np.isfinite(tr)
    ms, tr = ms[ok], tr[ok]
    if not len(ms):
        return None
    x = ms / 1000
    off = tr - x
    bins = np.floor((x - x.min()) / 30.0).astype(int)
    pts = [(x[bins == k][np.argmin(off[bins == k])], off[bins == k].min()) for k in np.unique(bins)]
    if len(pts) >= 3 and x.max() - x.min() > 60:
        px, po = np.array(pts).T
        slope, inter = np.polyfit(px, po, 1)
        resid = po - (inter + slope * px)
        keep = resid <= np.median(resid) + 0.01
        slope, inter = np.polyfit(px[keep], po[keep], 1) if keep.sum() >= 2 else (slope, inter)
        return float(inter), float(1.0 + slope), dict(source="arrival times (lower envelope)", n=len(pts))
    return float(min(p[1] for p in pts)), 1.0, dict(source="arrival times (lowest)", n=len(pts))


def laptop_to_wall_clock(recording=None, laptop_to_wall=None):
    """(LinearClock laptop -> t_wall, plain-words text, found?)."""
    if isinstance(laptop_to_wall, LinearClock):
        return laptop_to_wall, laptop_to_wall.text or "laptop -> camera clock as given", True
    if laptop_to_wall is not None:
        o = float(laptop_to_wall)
        return LinearClock(o, 1.0, "camera clock = laptop clock %+.0f ms (given)" % (o * 1000)), "", True
    p = os.path.join(recording, "video_clock.json") if recording else None
    if p and os.path.isfile(p):
        try:
            with open(p) as f:
                d = json.load(f)
            o = float(d["offset_s"])
            drift = float(d.get("drift_ppm") or 0.0) * 1e-6 if d.get("use_drift") else 0.0
            tref = float(d.get("t_ref") or 0.0)
            # t_wall = laptop + o + drift (laptop - tref)
            return LinearClock(o - drift * tref, 1.0 + drift,
                               "camera clock = laptop clock %+.0f ms (video_clock.json)" % (o * 1000)), "", True
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return LinearClock(0.0, 1.0, "camera clock assumed equal to the laptop clock (no video_clock.json: run "
                       "tools/check_session.py on the session, or use outside.estimate_time_offset)"), "", False


# =============================================================================== the plug-in
@register
class AlvikLogs(Plugin):
    id = "alvik"
    title = "Alvik robot logs"
    description = ("A console session folder (raw.csv and friends, tools/alvik_console.py): each robot's wheels "
                   "and gyro, placed into the picture.")

    def detect(self, folder):
        if not os.path.isdir(folder):
            return False
        for p in glob.glob(os.path.join(folder, "raw*.csv")):
            try:
                with open(p, newline="", encoding="utf-8", errors="replace") as f:
                    head = next(csv.reader(f), [])
            except OSError:
                continue
            if {"robot", "t_recv", "ms"} <= {h.strip() for h in head}:
                return True
        return False

    def read(self, folder, **options):
        return read_alvik_logs(folder, **options)


def read_alvik_logs(folder, recording=None, laptop_to_wall=None, names=None, use_gyro=True, offset_s=0.0, **_ignored):
    """Read a console session folder -> OutsideData (see the module docstring). names {log name:
    name to use} renames robots (e.g. AL-3F2A -> Left) after reading. offset_s: seconds added on top
    of the clock found (the person's correction, or one estimated from speed; t_wall = log time +
    offset_s). OutsideData.clock_known is False when there was no video_clock.json, no laptop_to_wall
    and no offset: the caller may then estimate one (outside.estimate_time_offset)."""
    notes = []
    raw, files, skipped = read_rows(folder, "raw")
    if not files:
        return OutsideData("alvik", {}, [], ["no raw.csv in %s: no robot positions" % folder], "", folder)
    if skipped:
        notes.append("%d damaged raw rows skipped" % skipped)
    runinfo, _, _ = read_rows(folder, "runinfo")
    if not runinfo:
        notes.append("no runinfo.csv: the heading's sign was not checked, so it is fitted from the anchors")
    signs = {r.get("run", ""): (_f(r.get("yaw_sign"), 0.0), _f(r.get("odo_sign"), 0.0)) for r in runinfo}
    params, _, _ = read_rows(folder, "param")
    corr = {}
    for r in params:
        if r.get("name") in ("gyro_scale", "wheel_scale", "gyro_drift_dps"):
            v = _f(r.get("value"))
            if math.isfinite(v):
                corr.setdefault(r.get("robot", ""), {})[r["name"]] = v

    # one name per boot: a robot renamed during the session (AL-3F2A -> Left) keeps its boot id
    boot_names = {}
    for r in raw:
        b = _boot(r.get("run"))
        if b:
            boot_names.setdefault(b, {}).setdefault(r.get("robot", ""), 0)
            boot_names[b][r.get("robot", "")] += 1
    rename = {}
    for b, cnt in boot_names.items():
        if len(cnt) > 1:
            main = max(cnt, key=cnt.get)
            for n in cnt:
                if n != main:
                    rename[n] = main
    if rename:
        notes.append("renamed during the session: " + ", ".join("%s -> %s" % kv for kv in sorted(rename.items())))
    for n_from, n_to in (names or {}).items():
        rename[n_from] = n_to
        for k, v in list(rename.items()):
            if v == n_from:
                rename[k] = n_to

    fits = clock_fits(folder)
    if not fits:
        notes.append("no clock_sync.csv: robot clocks set from the arrival times (a few ms less exact)")
    to_wall, _, found = laptop_to_wall_clock(recording, laptop_to_wall)
    offset_s = float(offset_s or 0.0)
    if offset_s:
        to_wall = LinearClock(to_wall.a + offset_s, to_wall.b, "%s; then %+.2f s" % (to_wall.text, offset_s))
    elif not found:
        notes.append(to_wall.text)
    clock_text = to_wall.text

    by_robot = {}
    for r in raw:
        name = r.get("robot", "")
        by_robot.setdefault(rename.get(name, name), []).append(r)

    streams, events = {}, []
    for name, rows in sorted(by_robot.items()):
        st, ev, nts = _robot_stream(name, rows, signs, fits, to_wall, corr.get(name) or {}, use_gyro,
                                    folder, rename)
        notes += nts
        if st is not None:
            streams[name] = st
        events += ev
    events += _events(folder, fits, to_wall, rename, by_robot)
    events.sort(key=lambda e: e["t"])
    return OutsideData("alvik", streams, events, notes, clock_text, folder, clock_known=bool(found or offset_s))


def _robot_clock(name, boot, rows, fits, rename):
    """robot ms -> laptop for one robot and boot."""
    for key in ((name, boot),) + tuple((old, boot) for old, new in rename.items() if new == name):
        if key in fits:
            return fits[key], "clock_sync.csv"
    same = [v for (r, b), v in fits.items() if (r == name or rename.get(r) == name) and b == ""]
    if len(same) == 1:
        return same[0], "clock_sync.csv"
    env = envelope_fit([_f(r.get("ms")) for r in rows], [_f(r.get("t_recv")) for r in rows])
    return env, "arrival times"


def _robot_stream(name, rows, signs, fits, to_wall, corr, use_gyro, folder, rename):
    notes = []
    # group by boot, in the order the boots were heard
    boots = {}
    for r in rows:
        boots.setdefault(_boot(r.get("run")), []).append(r)
    order = sorted(boots, key=lambda b: min(_f(r.get("t_recv"), 1e18) for r in boots[b]))
    T, D, H, SEG, VALID = [], [], [], [], []
    seg = 0
    sign_known = True
    dist_total, head_total = 0.0, 0.0
    used_clock = set()
    for bi, b in enumerate(order):
        rs = [r for r in boots[b] if math.isfinite(_f(r.get("ms")))]
        rs.sort(key=lambda r: _f(r.get("ms")))
        if not rs:
            continue
        fit, src = _robot_clock(name, b, rs, fits, rename)
        if fit is None:
            notes.append("%s boot %s: no clock; rows skipped" % (name, b or "?"))
            continue
        used_clock.add(src)
        a, k = fit[0], fit[1]
        if bi > 0:
            seg += 1          # a new boot: a new start frame
        prev = None
        prev_head_raw = None
        for r in rs:
            run = r.get("run", "")
            ys, os_ = signs.get(run, (0.0, 0.0))
            ms = _f(r.get("ms"))
            t = float(to_wall(a + k * ms / 1000.0))
            x, y, th = _f(r.get("x_cm")), _f(r.get("y_cm")), _f(r.get("th_deg"))
            yaw = _f(r.get("yaw_deg"))
            wl, wr = _f(r.get("wl_deg")), _f(r.get("wr_deg"))
            lifted = r.get("lifted", "0").strip() in ("1", "1.0", "true")
            # --- distance
            ds = 0.0
            new_run = prev is not None and run != prev["run"]
            if prev is not None:
                if math.isfinite(x) and math.isfinite(y):
                    px, py = prev["x"], prev["y"]
                    if new_run and (not (math.isfinite(px) and math.isfinite(py)) or
                                    math.hypot(x, y) < max(RUN_RESET_CM, math.hypot(x - px, y - py))):
                        # the library pose was reset at the run start (it starts nearer 0 than where the
                        # last run ended; the robot stands still between runs)
                        px, py = 0.0, 0.0
                    if math.isfinite(px) and math.isfinite(py):
                        dx, dy = x - px, y - py
                        step = math.hypot(dx, dy)
                        hh = th if math.isfinite(th) else prev["th"]
                        if math.isfinite(hh):
                            sgn = 1.0 if dx * math.cos(math.radians(hh)) + dy * math.sin(math.radians(hh)) >= 0 else -1.0
                        else:
                            sgn = 1.0
                        ds = sgn * step
                elif math.isfinite(wl) and math.isfinite(wr) and math.isfinite(prev["wl"]) and math.isfinite(prev["wr"]):
                    dw = ((wl - prev["wl"]) + (wr - prev["wr"])) / 2.0
                    if abs(dw) < 5000:
                        ds = dw * math.pi * WHEEL_DIAM_CM / 360.0
                gap = t - prev["t"]
                if gap > NEW_SEGMENT_GAP_S:
                    seg += 1
                    ds = 0.0
            # --- heading
            if use_gyro and math.isfinite(yaw):
                sgn_h = ys if ys else 1.0
                if not ys:
                    sign_known = False
                hraw = yaw * sgn_h
            elif math.isfinite(th):
                sgn_h = os_ if os_ else 1.0
                if not os_:
                    sign_known = False
                hraw = th * sgn_h
            else:
                hraw = math.nan
            dh = 0.0
            if prev_head_raw is not None and math.isfinite(hraw):
                dh = (hraw - prev_head_raw + 180.0) % 360.0 - 180.0
                if new_run and abs(dh) > RESET_TURN_DEG and abs((hraw + 180.0) % 360.0 - 180.0) < abs(dh):
                    # reset at the run start: it starts nearer 0 than where the last run ended (rows lost
                    # at a run start after a real turn keep the turn: then the old heading is nearer)
                    dh = (hraw + 180.0) % 360.0 - 180.0
            if math.isfinite(hraw):
                prev_head_raw = hraw
            if lifted:
                ds = 0.0
            dist_total += ds
            head_total += dh
            if prev is not None and prev["lifted"] and not lifted:
                seg += 1              # put down somewhere: a new segment
            T.append(t)
            D.append(dist_total)
            H.append(head_total)
            SEG.append(seg)
            VALID.append(not lifted)
            prev = dict(run=run, x=x, y=y, th=th, wl=wl, wr=wr, t=t, lifted=lifted)
    if not T:
        return None, [], notes + ["%s: no usable rows" % name]
    T = np.array(T)
    # rows can arrive out of order across boots: keep time order (stable)
    o = np.argsort(T, kind="stable")
    T, D, H = T[o], np.array(D)[o], np.array(H)[o]
    SEG, VALID = np.array(SEG)[o], np.array(VALID)[o]
    a = np.interp(T - STILL_WINDOW_S / 2, T, D)
    b = np.interp(T + STILL_WINDOW_S / 2, T, D)
    ha = np.interp(T - STILL_WINDOW_S / 2, T, H)
    hb = np.interp(T + STILL_WINDOW_S / 2, T, H)
    still = (np.abs(b - a) < STILL_CM) & (np.abs(hb - ha) < 0.5)
    priors = dict(ALVIK_PRIORS)
    if corr.get("gyro_scale") and abs(corr["gyro_scale"] - 1.0) > 1e-6:
        priors.update(g=corr["gyro_scale"], g_sd=0.02)
    if corr.get("wheel_scale") and abs(corr["wheel_scale"] - 1.0) > 1e-6:
        priors.update(s=corr["wheel_scale"], s_sd=0.01)
    if not sign_known:
        notes.append("%s: heading sign not checked on some runs (no compass check): fitted from the anchors" % name)
    if "arrival times" in used_clock:
        notes.append("%s: clock from arrival times (no clock_sync.csv answers for this robot)" % name)
    st = Stream(name, T, dist=D, heading=H, frame="own", segment=SEG, valid=VALID, still=still, cls="alvik",
                sign_known=sign_known, priors=priors,
                meta=dict(rows=len(T), segments=int(len(set(SEG.tolist()))), clock=sorted(used_clock)))
    return st, [], notes


def _events(folder, fits, to_wall, rename, by_robot):
    """dock.csv, posts.csv, runinfo/runend, sync.csv and lifted stretches as events on t_wall."""
    out = []
    env = {}

    def t_of(robot, run, ms):
        ms = _f(ms)
        if not math.isfinite(ms):
            return None
        name = rename.get(robot, robot)
        b = _boot(run)
        key = (name, b)
        if key not in env:
            rows = [r for r in by_robot.get(name, []) if _boot(r.get("run")) == b]
            env[key] = _robot_clock(name, b, rows, fits, rename)[0] if (rows or fits) else None
        fit = env[key]
        if fit is None:
            return None
        return float(to_wall(fit[0] + fit[1] * ms / 1000.0))

    for r in read_rows(folder, "dock")[0]:
        t = t_of(r.get("robot", ""), r.get("run"), r.get("ms_both") or r.get("ms_first"))
        if t is not None:
            out.append(dict(t=t, kind="docked" if r.get("result") == "docked" else "dock_failed",
                            name=rename.get(r.get("robot", ""), r.get("robot", "")), run=r.get("run"),
                            result=r.get("result"), still_ms=_f(r.get("still_ms"))))
    for r in read_rows(folder, "posts")[0]:
        t = t_of(r.get("robot", ""), r.get("run"), r.get("ms_first"))
        if t is not None:
            out.append(dict(t=t, kind="post_touch", name=rename.get(r.get("robot", ""), r.get("robot", "")),
                            run=r.get("run"), result=r.get("result")))
    for r in read_rows(folder, "runinfo")[0]:
        t = t_of(r.get("robot", ""), r.get("run"), r.get("ms_start"))
        if t is not None:
            out.append(dict(t=t, kind="run_start", name=rename.get(r.get("robot", ""), r.get("robot", "")),
                            run=r.get("run"), experiment=r.get("experiment")))
    for r in read_rows(folder, "runend")[0]:
        t = t_of(r.get("robot", ""), r.get("run"), r.get("ms_end"))
        if t is not None:
            out.append(dict(t=t, kind="run_end", name=rename.get(r.get("robot", ""), r.get("robot", "")),
                            run=r.get("run"), result=r.get("result")))
    for r in read_rows(folder, "sync")[0]:
        t = t_of(r.get("robot", ""), r.get("run"), r.get("ms_on"))
        t2 = t_of(r.get("robot", ""), r.get("run"), r.get("ms_off"))
        if t is not None:
            out.append(dict(t=t, kind="flash", name=rename.get(r.get("robot", ""), r.get("robot", "")),
                            run=r.get("run"), tag=r.get("tag"), t_off=t2))
    for name, rows in by_robot.items():
        was = False
        for r in sorted(rows, key=lambda r: (_f(r.get("t_recv"), 0.0))):
            lf = r.get("lifted", "0").strip() in ("1", "1.0", "true")
            if lf != was:
                t = t_of(name, r.get("run"), r.get("ms"))
                if t is not None:
                    out.append(dict(t=t, kind="lifted" if lf else "put_down", name=name, run=r.get("run")))
                was = lf
    return out
