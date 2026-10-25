"""Each robot's battery drain, learned from the logs: % per hour while moving, while still (awake) and while asleep,
each with its uncertainty; and a flag when a robot's drain changes (Nick, 2026-10-25: "connect the battery log with
the charging").

    python tools/dock/battery_rates.py                      every session on disk (DEFAULT_SESSIONS), per robot
    python tools/dock/battery_rates.py <session folder> ...  the given session folders
    python tools/dock/battery_rates.py --levels              ... and the go / leave levels those rates give
                                                             (tools/dock/charge_levels.py)

Where the readings come from, per session folder:
  battery.csv (the runner, every --battery-watch-s seconds, from 2026-10-24 night): t, robot, role, state (moving /
      still), still_s, battery, phase, source;
  the robot logs (robot-logs-*/): runinfo, runend, find, e02, gyro and the RAW rows of raw.csv carry a battery % too,
      and raw.csv's wheel speeds and leg.csv's legs say when each robot drove (the night analyses' method,
      tools/experiment/analyses_oct24/night_latency_battery.py);
  e02_log.txt: "<robot> is not in this session: sent to sleep" marks a robot asleep from then on.

The model, per robot (all its sessions together; the gauge is a whole %, so readings are first reduced to a median a
minute):
    battery = b(segment) - still rate x hours still - moving rate x hours moving - asleep rate x hours asleep
A segment is a stretch with no charging in it: a new one starts at a session's start (unless the session follows
the previous one within JOIN_GAP_MIN with no rise in the reading: then the robots stood switched on in between, as on
2026-10-24), wherever the reading rises by RISE_SPLIT or more, and wherever the robot reports charging. Least squares
gives the rates; their standard errors are inflated for the readings' correlation in time (lag-1 autocorrelation of
the residuals, the usual AR(1) factor sqrt((1 + rho) / (1 - rho))). A state with less than MIN_HOURS in the data has
no rate of its own and falls back (below).

Which rate is used (BatteryModel.rates): this session's own, once it has SESSION_MIN_HOURS of that state; else the
robot's earlier sessions (logs/battery_learned.json, pooled with their between-session spread); else the fleet
default (MEASURED in the night analyses; asleep is a PLACEHOLDER until the sleep test). The moment this session's own
rate is first known, it is compared with the earlier one (as tools/experiment/e02_front.py does for the front sensor):
a difference beyond max(SHIFT_MIN, SHIFT_SE x its standard error) is reported ("drains faster / slower than before").

Tests: tools/dock/tests/test_battery_rates.py.
"""
import csv
import glob
import json
import math
import os
import re
import sys

STATES = ("still", "moving", "asleep")
# fleet defaults, %/h (rate, standard error) ---------------------------------------------------------------
DEFAULTS = {
    "still": (14.0, 2.0),     # MEASURED: night analyses section C, standing switched on 11-16 %/h (3 robots, 3 days)
    "moving": (18.0, 5.0),    # MEASURED, loosely: day fits including driving 13-24 %/h; driving's own cost was too
                              # small to separate (robots drove a quarter of the time)
    "asleep": (3.0, 3.0),     # PLACEHOLDER: firmware `sleep` not measured yet (test: one robot asleep 1 h)
}
MOVING_RPM = 5.0              # chosen (as the night analyses): |left rpm| + |right rpm| above this counts as driving
RISE_SPLIT = 3.0              # chosen: a rise of 3 % (the gauge's smoothing never rises by more than 1) = a charge
JOIN_GAP_MIN = 45.0           # chosen: sessions this close, with no rise, are one segment (robots left on between)
MIN_HOURS = 0.1               # chosen: less exposure than this to a state: no rate of its own (6 minutes moves an
                              # integer gauge by about 1-2 %)
SESSION_MIN_HOURS = {"still": 0.75, "moving": 0.25, "asleep": 0.75}   # chosen: before this session's own rate replaces
                              # the earlier one. DOE note: replay the sessions on disk at 0.25 / 0.5 / 1 h and count
                              # how soon a changed rate is caught against false "changed" reports (doe-backlog.md)
SHIFT_MIN = 3.0               # chosen: a change smaller than 3 %/h is not reported
SHIFT_SE = 3.0                # ... nor one within 3 standard errors
PRIOR_KEEP = 12               # sessions kept per robot in the history file
TZ_H = -4.0                   # the sessions' local time (EDT) for the log's [HH:MM:SS] stamps
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
HISTORY = os.path.join(REPO, "logs", "battery_learned.json")
ROOT = os.environ.get("NIGHT_SESSIONS_ROOT", "/tmp/claude-0")
DEFAULT_SESSIONS = [os.path.join(ROOT, p) for p in (
    "oct23/e02-20261023-085107", "evening/e02-20261023-191347", "plan/153612/e02-20261024-153612",
    "plan/161612/e02-20261024-161612", "plan/173242/e02-20261024-173242", "r3/logs/e02-20261024-193526")]


def _num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def _rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))


def _merge(iv):
    out = []
    for a, b in sorted(iv):
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def _overlap(iv, t0, t1):
    return sum(max(0.0, min(b, t1) - max(a, t0)) for a, b in iv)


def session_start(folder):
    """Unix seconds of the folder name's e02-YYYYMMDD-HHMMSS (local time, TZ_H), or None."""
    m = re.search(r"(\d{8})-(\d{6})", os.path.basename(os.path.normpath(folder)))
    if not m:
        return None
    import calendar
    d, t = m.group(1), m.group(2)
    return calendar.timegm((int(d[:4]), int(d[4:6]), int(d[6:]), int(t[:2]), int(t[2:4]), int(t[4:]))) - TZ_H * 3600


def load_session(folder):
    """One session folder -> dict(name, readings {robot: [(t, %)]}, charging {robot: [t]}, moving {robot: [(t0, t1)]},
    asleep {robot: [(t0, t1)]}, t0, t1, sources [what was read])."""
    rd, chg, mov, slp, src = {}, {}, {}, {}, []
    bat = _rows(os.path.join(folder, "battery.csv"))
    if bat:
        src.append("battery.csv (%d rows)" % len(bat))
    last = {}
    has_raw = bool(glob.glob(os.path.join(folder, "robot-logs-*", "raw.csv")))
    for r in sorted(bat, key=lambda r: _num(r.get("t")) or 0):
        t, b, n = _num(r.get("t")), _num(r.get("battery")), r.get("robot")
        if t is None or b is None or b <= 0 or not n:
            continue
        rd.setdefault(n, []).append((t, b))
        st = (r.get("state") or "").strip()
        t_prev = last.get(n)
        if st in ("asleep", "sleep") and t_prev is not None:
            slp.setdefault(n, []).append((t_prev, t))
        elif not has_raw and t_prev is not None:
            # battery.csv alone (no RAW rows to say when the wheels turned): ASSUMED half of the time the robot was
            # not known to be still. A moving row: half the interval; a still row with still_s shorter than the
            # interval: half of the part before it came to rest
            dt = t - t_prev
            still_s = min(dt, _num(r.get("still_s")) or 0.0)
            if st == "moving":
                mov.setdefault(n, []).append((t - dt / 2, t))
            elif still_s < dt - 1.0:
                mov.setdefault(n, []).append((t - still_s - (dt - still_s) / 2, t - still_s))
        last[n] = t
    logs = sorted(glob.glob(os.path.join(folder, "robot-logs-*")))
    for d in logs:
        for fn in ("runinfo.csv", "runend.csv", "find.csv", "e02.csv", "gyro.csv"):
            for r in _rows(os.path.join(d, fn)):
                t, b = _num(r.get("t_recv")), _num(r.get("battery"))
                if t is not None and b is not None and b > 0 and r.get("robot"):
                    rd.setdefault(r["robot"], []).append((t, b))
                    if _num(r.get("charging")):
                        chg.setdefault(r["robot"], []).append(t)
        raw = _rows(os.path.join(d, "raw.csv"))
        cur = {}
        for r in sorted(raw, key=lambda r: _num(r.get("t_recv")) or 0):
            n, t = r.get("robot"), _num(r.get("t_recv"))
            if not n or t is None:
                continue
            b = _num(r.get("battery"))
            if b is not None and b > 0:
                rd.setdefault(n, []).append((t, b))
            if _num(r.get("charging")):
                chg.setdefault(n, []).append(t)
            wl, wr = _num(r.get("wl_rpm")), _num(r.get("wr_rpm"))
            if wl is None:
                continue
            if abs(wl) + abs(wr or 0.0) > MOVING_RPM:
                c = cur.get(n)
                if c and t - c[1] < 0.2:
                    c[1] = t
                else:
                    if c:
                        mov.setdefault(n, []).append(tuple(c))
                    cur[n] = [t, t]
        for n, c in cur.items():
            mov.setdefault(n, []).append(tuple(c))
        started = {}
        for r in _rows(os.path.join(d, "leg.csv")):
            if r.get("run") not in ("-", ""):
                continue
            key, t = (r.get("robot"), r.get("id")), _num(r.get("t_recv"))
            if r.get("result") == "started":
                started[key] = t
            elif r.get("result") in ("done", "failed", "stopped") and key in started and t is not None:
                mov.setdefault(r["robot"], []).append((started.pop(key), t))
        if raw:
            src.append("%s (raw %d rows)" % (os.path.basename(d), len(raw)))
    t_start = session_start(folder)
    log = os.path.join(folder, "e02_log.txt")
    if os.path.exists(log) and t_start is not None:
        day0 = t_start - ((t_start + TZ_H * 3600) % 86400)
        with open(log, encoding="utf-8", errors="replace") as f:
            for ln in f:
                m = re.match(r"\[(\d\d):(\d\d):(\d\d)\]\s+(\w+) is not in this session: sent to sleep", ln)
                if m:
                    t = day0 + int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))
                    if t < t_start - 600:
                        t += 86400
                    slp.setdefault(m.group(4), []).append((t, float("inf")))
    allt = [t for v in rd.values() for t, _ in v]
    return dict(name=os.path.basename(os.path.normpath(folder)), readings={k: sorted(v) for k, v in rd.items()},
                charging={k: sorted(v) for k, v in chg.items()}, moving={k: _merge(v) for k, v in mov.items()},
                asleep={k: _merge(v) for k, v in slp.items()}, t0=min(allt) if allt else None,
                t1=max(allt) if allt else None, sources=src)


def per_minute(readings):
    by = {}
    for t, b in readings:
        by.setdefault(int(t // 60), []).append(b)
    out = []
    for k in sorted(by):
        v = sorted(by[k])
        n = len(v)
        out.append((k * 60.0 + 30.0, v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])))
    return out


def samples(sessions, robot):
    """[(segment id, t, %, still h, moving h, asleep h)] for one robot over sessions (oldest first): the
    cumulative hours in each state since its segment began."""
    out = []
    seg = -1
    prev = None          # (t, %, still, moving, asleep, session index) of the last sample
    for si, s in enumerate(sorted(sessions, key=lambda s: s["t0"] or 0)):
        pm = per_minute(s["readings"].get(robot, []))
        if not pm:
            continue
        mov, slp, chg = s["moving"].get(robot, []), s["asleep"].get(robot, []), s["charging"].get(robot, [])
        for t, b in pm:
            if any(abs(tc - t) <= 60.0 for tc in chg):
                prev = None          # charging: drop it; the next reading starts a new segment
                continue
            new = prev is None
            if prev is not None:
                gap_min = (t - prev[0]) / 60.0
                if b - prev[1] >= RISE_SPLIT:
                    new = True
                elif prev[5] != si and gap_min > JOIN_GAP_MIN:
                    new = True
            if new:
                seg += 1
                cum = [0.0, 0.0, 0.0]
            else:
                t0 = prev[0]
                mv = _overlap(mov, t0, t) if prev[5] == si else 0.0
                sl = max(0.0, min(t - t0 - mv, _overlap(slp, t0, t))) if prev[5] == si else 0.0
                cum = [prev[2] + (t - t0 - mv - sl) / 3600.0, prev[3] + mv / 3600.0, prev[4] + sl / 3600.0]
            out.append((seg, t, b, cum[0], cum[1], cum[2]))
            prev = (t, b, cum[0], cum[1], cum[2], si)
    return out


class Rate:
    __slots__ = ("rate", "se", "hours", "source")

    def __init__(self, rate, se, hours, source):
        self.rate, self.se, self.hours, self.source = float(rate), float(se), float(hours), source

    def hi(self, k=2.0):
        return self.rate + k * self.se

    def as_list(self):
        return [round(self.rate, 3), round(self.se, 3), round(self.hours, 3)]

    def __repr__(self):
        return "%.1f +- %.1f %%/h (%.2f h, %s)" % (self.rate, self.se, self.hours, self.source)


def fit(smp):
    """Least squares on samples() -> ({state: Rate or None}, info). info: n, segments, rms, rho, moving share."""
    import numpy as np
    if len(smp) < 4:
        return {k: None for k in STATES}, dict(n=len(smp), segments=0, rms=None, rho=None, moving_share=None)
    segs = sorted(set(r[0] for r in smp))
    tot = {k: 0.0 for k in STATES}
    last = {}
    for r in smp:
        last[r[0]] = r
    for r in last.values():
        for i, k in enumerate(STATES):
            tot[k] += r[3 + i]
    cols = [k for k in STATES if tot[k] >= MIN_HOURS]
    X = []
    for r in smp:
        row = [1.0 if r[0] == s else 0.0 for s in segs]
        row += [-r[3 + STATES.index(k)] for k in cols]
        X.append(row)
    X, y = np.array(X), np.array([r[2] for r in smp], dtype=float)
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ beta
    dof = max(1, len(y) - X.shape[1])
    s2 = max(float(res @ res) / dof, 1.0 / 12.0)          # at least the whole-% rounding's variance
    rho = 0.0
    pairs = [(res[i], res[i + 1]) for i in range(len(res) - 1) if smp[i][0] == smp[i + 1][0]]
    if len(pairs) > 5:
        a = np.array(pairs)
        den = float(np.sum(a[:, 0] ** 2))
        rho = max(0.0, min(0.95, float(np.sum(a[:, 0] * a[:, 1])) / den)) if den > 0 else 0.0
    infl = math.sqrt((1 + rho) / (1 - rho))
    try:
        cov = s2 * np.linalg.inv(X.T @ X)
    except np.linalg.LinAlgError:
        cov = s2 * np.linalg.pinv(X.T @ X)
    out = {k: None for k in STATES}
    for j, k in enumerate(cols):
        i = len(segs) + j
        se = math.sqrt(max(cov[i, i], 0.0)) * infl
        out[k] = Rate(beta[i], se, tot[k], "fit")
    share = tot["moving"] / (tot["moving"] + tot["still"]) if tot["moving"] + tot["still"] > 0 else None
    return out, dict(n=len(smp), segments=len(segs), rms=math.sqrt(float(res @ res) / len(y)), rho=rho,
                     moving_share=share)


def fit_sessions(sessions, robots=None):
    robots = robots or sorted({r for s in sessions for r in s["readings"]})
    return {r: fit(samples(sessions, r)) for r in robots}


def pooled(entries):
    """Earlier sessions' estimates [(rate, se, hours)] -> Rate: inverse-variance mean, its se widened by the spread
    between sessions (a robot's drain moves from day to day: battery age, temperature, what it did)."""
    e = [x for x in entries if x and x[1] > 0]
    if not e:
        return None
    w = [1.0 / x[1] ** 2 for x in e]
    m = sum(wi * x[0] for wi, x in zip(w, e)) / sum(w)
    se_m = math.sqrt(1.0 / sum(w))
    tau2 = 0.0
    if len(e) > 1:
        var = sum((x[0] - m) ** 2 for x in e) / (len(e) - 1)
        tau2 = max(0.0, var - sum(x[1] ** 2 for x in e) / len(e))
    return Rate(m, math.sqrt(se_m ** 2 + tau2), sum(x[2] for x in e), "earlier sessions (%d)" % len(e))


class BatteryModel:
    """Per robot: the rate to use for each state, this session's against the history, and change flags.

        m = BatteryModel()                       # loads logs/battery_learned.json
        words = m.update(load_session(dir))      # call every few minutes in a session: [messages]
        m.rates("Left")["still"].hi()            # the upper bound used for the go level
        m.save()                                 # at the end: this session joins the history
    """

    def __init__(self, path=HISTORY):
        self.path = path
        self.history = {}
        self.session = {}
        self.info = {}
        self.session_id = None
        self.reported = set()
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    self.history = (json.load(f).get("robots") or {})
            except (OSError, ValueError):
                self.history = {}

    def prior(self, robot, state):
        ent = [e.get(state) for e in (self.history.get(robot) or [])[-PRIOR_KEEP:]
               if e.get("id") != self.session_id]
        return pooled([x for x in ent if x and x[2] >= MIN_HOURS])

    def rates(self, robot):
        out = {}
        for k in STATES:
            s = (self.session.get(robot) or {}).get(k)
            if s is not None and s.hours >= SESSION_MIN_HOURS[k]:
                out[k] = Rate(s.rate, s.se, s.hours, "this session")
                continue
            p = self.prior(robot, k)
            out[k] = p if p is not None else Rate(DEFAULTS[k][0], DEFAULTS[k][1], 0.0, "fleet default")
        return out

    def moving_share(self, robot, default=0.25):
        sh = (self.info.get(robot) or {}).get("moving_share")
        if sh is None:
            hist = [e.get("moving_share") for e in (self.history.get(robot) or []) if e.get("moving_share") is not None]
            sh = sum(hist) / len(hist) if hist else default
        return sh

    def update(self, session):
        """Refit this session's own rates from its data so far; return change messages (each once)."""
        self.session_id = session["name"]
        words = []
        for robot, (r, info) in fit_sessions([session]).items():
            self.session[robot], self.info[robot] = r, info
            for k in STATES:
                s = r.get(k)
                if s is None or s.hours < SESSION_MIN_HOURS[k] or (robot, k) in self.reported:
                    continue
                p = self.prior(robot, k)
                self.reported.add((robot, k))
                if p is None:
                    continue
                diff = s.rate - p.rate
                se = math.hypot(s.se, p.se)
                if abs(diff) > max(SHIFT_MIN, SHIFT_SE * se):
                    words.append("%s drains %s while %s than before: %.1f %%/h this session (%.1f h) against %.1f "
                                 "(%s); this session's rate is used" % (
                                     robot, "faster" if diff > 0 else "slower", k, s.rate, s.hours, p.rate, p.source))
        return words

    def save(self):
        if not self.path or not self.session_id:
            return
        for robot, r in self.session.items():
            ent = dict(id=self.session_id, moving_share=(self.info.get(robot) or {}).get("moving_share"))
            for k in STATES:
                if r.get(k) is not None:
                    ent[k] = r[k].as_list()
            h = [e for e in (self.history.get(robot) or []) if e.get("id") != self.session_id] + [ent]
            self.history[robot] = h[-PRIOR_KEEP:]
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(dict(robots=self.history), f, indent=1)
        os.replace(tmp, self.path)


def replay(folders, history_path=None):
    """Feed the sessions in time order through BatteryModel as the runner would (each one: update, then save), so
    each is judged against the ones before. Returns [(session, robot, {state: Rate}, info, [messages])]."""
    sess = sorted((load_session(f) for f in folders), key=lambda s: s["t0"] or 0)
    m = BatteryModel(history_path)
    out = []
    for s in sess:
        if s["t0"] is None:
            continue
        words = m.update(s)
        for robot in sorted(m.session):
            if robot in s["readings"]:
                out.append((s["name"], robot, dict(m.session[robot]), dict(m.info[robot]),
                            [w for w in words if w.startswith(robot)]))
        m.save()
        m.session, m.info = {}, {}
    return out, m


def _fmt(r):
    return "-" if r is None else "%.1f +- %.1f (%.2f h)" % (r.rate, r.se, r.hours)


def main(argv=None):
    import argparse
    import tempfile
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="*")
    ap.add_argument("--levels", action="store_true", help="also print the go / leave levels from these rates")
    a = ap.parse_args(argv)
    folders = [f for f in (a.folders or DEFAULT_SESSIONS) if os.path.isdir(f)]
    if not folders:
        print("no session folders found")
        return 1
    sess = [load_session(f) for f in folders]
    print("sessions:")
    for s in sorted(sess, key=lambda s: s["t0"] or 0):
        print("  %s: %s; robots %s" % (s["name"], ", ".join(s["sources"]) or "nothing", ", ".join(sorted(s["readings"]))))
    print("\nall sessions together, per robot (%%/h, +- one standard error, hours of data in that state):")
    print("| robot | still (awake) | moving | asleep | readings (minutes) | segments | rms % | rho | moving share |")
    print("|---|---|---|---|---|---|---|---|---|")
    allfit = fit_sessions(sess)
    for robot, (r, info) in sorted(allfit.items()):
        print("| %s | %s | %s | %s | %d | %d | %s | %s | %s |" % (
            robot, _fmt(r["still"]), _fmt(r["moving"]), _fmt(r["asleep"]), info["n"], info["segments"],
            "-" if info["rms"] is None else "%.2f" % info["rms"], "-" if info["rho"] is None else "%.2f" % info["rho"],
            "-" if info["moving_share"] is None else "%.2f" % info["moving_share"]))
    print("\nsession by session, as the runner would see them (each judged against the ones before):")
    with tempfile.TemporaryDirectory() as td:
        rows, model = replay(folders, os.path.join(td, "h.json"))
        print("| session | robot | still | moving | asleep | flags |")
        print("|---|---|---|---|---|---|")
        for name, robot, r, info, words in rows:
            print("| %s | %s | %s | %s | %s | %s |" % (name, robot, _fmt(r.get("still")), _fmt(r.get("moving")),
                                                     _fmt(r.get("asleep")), "; ".join(words) or ""))
        if a.levels:
            import charge_levels as CL
            robots = sorted(model.history)
            print("\ngo / leave levels from the learned rates (history of all sessions above), %d robots, one dock, "
                  "nobody at the dock:" % len(robots))
            for robot in robots:
                lv = CL.levels(robot, model, fleet=[dict(robot=o, state="working") for o in robots if o != robot])
                print("  %s: go %.0f %%, leave %.0f %%, early top-up below %.0f %%" % (robot, lv["go"], lv["leave"],
                                                                                    lv["topup"]))
                for w in lv["reasons"]:
                    print("      " + w)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    raise SystemExit(main())
