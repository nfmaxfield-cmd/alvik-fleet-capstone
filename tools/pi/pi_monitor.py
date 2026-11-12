#!/usr/bin/env python3
"""The Pi efficiency monitor (Mission Control build, Nick 12 Nov 2026: "we need to keep a close eye on this; come up
with a way of monitoring Pi efficiency"). The trained detector now runs on demand on the planner's core 1, capped at
25 % of a core (tools/fleet/tagfree.py); this watches what that costs, on every run.

    python3 tools/pi/pi_monitor.py watch --out RUN_FOLDER [--pgid N | --pid N] [--newest 'logs/*-freeroam']
    python3 tools/pi/pi_monitor.py report RUN_FOLDER          # pi_monitor.png + pi_monitor.md (the run report's section)
    python3 tools/pi/pi_monitor.py now                         # one reading, for a terminal

watch writes RUN_FOLDER/pi_monitor.csv, one row every EVERY_S, until the run (its process group, or a pid) has ended:
  t, clock                      Unix time, local time
  core0..core3                  each core's busy share since the last row (%, /proc/stat)
  load1                         the 1-minute load average
  temp_c                        SoC temperature (/sys/class/thermal/thermal_zone0/temp)
  throttled, throttle_now, throttle_since_boot
                                `vcgencmd get_throttled` (hex, then its bits in words); blank where there is no
                                vcgencmd (not a Pi): the row is still written
  mem_avail_mb, mem_total_mb    /proc/meminfo
  disk_free_gb                  free space where the run folder is (shutil.disk_usage)
  lag_overhead_s, lag_side_s    picture lag per camera: now minus the newest picture's capture time, read from the
                                recorder's /live header (X-Time) on 8000 / 8001 (one small low-quality picture a camera
                                a row); blank when the recorder is not running (practice)
  held_overhead_s, held_side_s  the newest picture's age as the PLANNER holds it (monitor_feed.json, free roam)
  loop_work_ms_p50, loop_work_ms_p95, loop_work_ms_max, loop_period_ms
                                the planner's loop: its work a pass and its period (monitor_feed.json); free roam
                                aims at 50 ms a pass (freeroam.TICK_S)
  det_calls, det_ms_p50, det_ms_p95, det_share_pct, det_cap_pct, det_refused, det_side_off
                                the detector (tagfree's own numbers via monitor_feed.json): calls so far, ms a call,
                                its share of its core over the last 4 s against its cap, calls refused for load
Nothing here can stop or slow a run: every read is guarded, the process runs at low priority (nice 10).

Warnings (health()) come from the run's own data or the Pi's documented limits, never a typed-in number:
  temperature   the Pi's documented throttle point, 80 C (Raspberry Pi 4 and 5 firmware: the ARM clock is cut from 80 C,
                more at 85 C); and the firmware's own flags: any throttle flag NOW is red, "since boot" is a note
  picture lag   red over the robots' own limit for a fix (params.h fix_max_age_ms: 1 s; an older fix is thrown away);
                amber over this run's own 95th percentile so far (after WARMUP_ROWS rows)
  loop          red when a pass takes longer than its period target (freeroam.TICK_S 50 ms: the loop falls behind);
                amber over this run's own 95th percentile
  a core        red when it is full [chosen: 99.5 % or more, two rows running]; amber over the run's own 95th percentile
  disk          red under the card floor at which runs stop (diskguard.card_stop_gb, per site); amber under twice it
                [chosen: x 2]
  memory        red under 5 % of the total available [chosen]
  detector      amber when its share of its core is over its own cap (its Budget then refuses calls until it is back
                under: a call can carry it over for a moment)

Changelog
  2026-11-12a  first version (Mission Control build)
"""
import argparse
import csv
import glob
import json
import os
import shutil
import socket
import statistics
import subprocess
import sys
import time

VERSION = "pi-monitor-2026-11-12a"
HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "experiment")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

EVERY_S = 2.0            # chosen: a row every 2 s, as cpu.csv (e02_cpulog). No DOE: a log's resolution
WARMUP_ROWS = 30         # chosen: a minute of rows before "this run's own 95th percentile" is used for amber. No DOE:
                         # a display rule (the red limits apply from the first row)
CAMERAS = (("overhead", 8000), ("side", 8001))   # the recorder's /live ports (camrec.py; mission_control.REAL_CAMERA_PORTS)
TEMP_LIMIT_C = 80.0      # DOCUMENTED: Raspberry Pi 4 / 5 firmware throttles the ARM clock from 80 C
FIX_MAX_AGE_S = 1.0      # DOCUMENTED in the firmware: params.h fix_max_age_ms 1000 (a camera fix older than this is not
                         # used by the robot)
LOOP_TARGET_MS = 50.0    # the planner's own period target: freeroam.TICK_S = 0.05
CORE_FULL_PCT = 99.5     # chosen: a core this busy is full (it rounds to 100 %). No DOE: a definition
DISK_AMBER_X = 2.0       # chosen: amber under twice the card floor. No DOE: a warning margin
MEM_RED_FRAC = 0.05      # chosen: red when under 5 % of memory is available. No DOE: a guard
THROTTLE_SYS = ("/sys/devices/platform/soc/soc:firmware/get_throttled",
                "/sys/devices/platform/soc/soc:firmware/raspberrypi-hwmon/get_throttled")
BITS = {0: "under-voltage", 1: "frequency capped", 2: "throttled", 3: "soft temperature limit"}
NAME = "pi_monitor.csv"
COLS = ["t", "clock", "core0", "core1", "core2", "core3", "load1", "temp_c", "throttled", "throttle_now",
        "throttle_since_boot", "mem_avail_mb", "mem_total_mb", "disk_free_gb", "lag_overhead_s", "lag_side_s",
        "held_overhead_s", "held_side_s", "loop_work_ms_p50", "loop_work_ms_p95", "loop_work_ms_max", "loop_period_ms",
        "det_calls", "det_ms_p50", "det_ms_p95", "det_share_pct", "det_cap_pct", "det_refused", "det_side_off"]


# ------------------------------------------------------------------------------------------------ readings
def read_stat(path="/proc/stat"):
    out = {}
    try:
        with open(path) as f:
            for ln in f:
                if ln.startswith("cpu") and ln[3:4].isdigit():
                    p = ln.split()
                    v = [int(x) for x in p[1:]]
                    idle = v[3] + (v[4] if len(v) > 4 else 0)
                    out[int(p[0][3:])] = (sum(v) - idle, sum(v))
    except (OSError, ValueError):
        pass
    return out


def core_loads(a, b):
    out = {}
    for k in b:
        if k in a:
            db, dt = b[k][0] - a[k][0], b[k][1] - a[k][1]
            out[k] = round(100.0 * db / dt, 1) if dt > 0 else 0.0
    return out


def _run(cmd, timeout=2):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout
    except (OSError, subprocess.SubprocessError):
        return 127, ""


def throttle(run=None, sys_files=THROTTLE_SYS):
    """(hex or None, [flags now], [flags since boot]); None where neither vcgencmd nor the sysfs file exists."""
    v = None
    for p in sys_files:
        try:
            with open(p) as f:
                v = int(f.read().strip(), 16)
            break
        except (OSError, ValueError):
            continue
    if v is None:
        rc, txt = (run or _run)(["vcgencmd", "get_throttled"])
        if rc == 0 and "=" in txt:
            try:
                v = int(txt.strip().split("=")[-1], 16)
            except ValueError:
                v = None
    if v is None:
        return None, [], []
    return hex(v), [w for b, w in BITS.items() if v & (1 << b)], [w for b, w in BITS.items() if v & (1 << (b + 16))]


def temp_c(path="/sys/class/thermal/thermal_zone0/temp"):
    try:
        with open(path) as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def meminfo(path="/proc/meminfo"):
    d = {}
    try:
        with open(path) as f:
            for ln in f:
                k, _, v = ln.partition(":")
                d[k] = int(v.split()[0]) / 1024.0
    except (OSError, ValueError, IndexError):
        pass
    return d.get("MemAvailable"), d.get("MemTotal")


def load1():
    try:
        return round(os.getloadavg()[0], 2)
    except OSError:
        return None


def picture_lag(port, host="127.0.0.1", timeout=1.0, clock=time.time):
    """now minus the newest picture's capture time (the recorder's X-Time), s; None when nothing answers. Asks for the
    lowest quality (q=30): one small encode."""
    try:
        c = socket.create_connection((host, port), timeout=timeout)
    except OSError:
        return None
    try:
        c.settimeout(timeout)
        c.sendall(b"GET /live?q=30 HTTP/1.0\r\nHost: x\r\n\r\n")
        buf = b""
        t0 = clock()
        while clock() - t0 < timeout and len(buf) < 262144:
            ch = c.recv(8192)
            if not ch:
                break
            buf += ch
            i = buf.find(b"X-Time: ")
            if i >= 0:
                j = buf.find(b"\r\n", i)
                if j > i:
                    return round(clock() - float(buf[i + 8:j]), 3)
        return None
    except (OSError, ValueError):
        return None
    finally:
        c.close()


def read_feed(folder):
    for p in (os.path.join(folder, "monitor_feed.json"), os.path.join(folder, "run", "monitor_feed.json")):
        try:
            with open(p) as f:
                d = json.load(f)
            if time.time() - float(d.get("t", 0)) < 30.0:
                return d
        except (OSError, ValueError, TypeError):
            continue
    return {}


def card_floor_gb():
    try:
        import diskguard as DG
        return float(DG.card_stop_gb())
    except Exception:  # noqa: BLE001
        return 1.0


# ------------------------------------------------------------------------------------------------ one row
class Sampler:
    def __init__(self, folder, cameras=CAMERAS, lag_fn=picture_lag, clock=time.time):
        self.folder, self.cameras, self.lag_fn, self.clock = folder, cameras, lag_fn, clock
        self.prev = read_stat()

    def row(self):
        now = self.clock()
        st = read_stat()
        cl = core_loads(self.prev, st)
        self.prev = st
        hx, tn, tb = throttle()
        ma, mt = meminfo()
        try:
            free = round(shutil.disk_usage(self.folder).free / 1e9, 2)
        except OSError:
            free = None
        r = {"t": round(now, 2), "clock": time.strftime("%H:%M:%S", time.localtime(now)), "load1": load1(),
             "temp_c": temp_c(), "throttled": hx or "", "throttle_now": "; ".join(tn),
             "throttle_since_boot": "; ".join(tb), "mem_avail_mb": round(ma) if ma else None,
             "mem_total_mb": round(mt) if mt else None, "disk_free_gb": free}
        for k in range(4):
            r["core%d" % k] = cl.get(k)
        for cam, port in self.cameras:
            r["lag_%s_s" % cam] = self.lag_fn(port)
        fd = read_feed(self.folder)
        cams = fd.get("cameras") or {}
        for cam, _ in self.cameras:
            r["held_%s_s" % cam] = cams.get(cam)
        lp = fd.get("loop") or {}
        r["loop_work_ms_p50"], r["loop_work_ms_p95"] = lp.get("work_ms_p50"), lp.get("work_ms_p95")
        r["loop_work_ms_max"], r["loop_period_ms"] = lp.get("work_ms_max"), lp.get("period_ms_mean")
        dt = fd.get("detector") or {}
        ld = dt.get("load") or {}
        r["det_calls"], r["det_ms_p50"], r["det_ms_p95"] = ld.get("calls"), ld.get("ms_median"), ld.get("ms_p95")
        r["det_share_pct"] = None if dt.get("share_now") is None else round(100.0 * dt["share_now"], 1)
        r["det_cap_pct"] = None if ld.get("duty") is None else round(100.0 * ld["duty"], 1)
        r["det_refused"], r["det_side_off"] = ld.get("refused_for_load"), dt.get("side_off")
        return r


def group_alive(pgid=None, pid=None):
    if pid:
        try:
            with open("/proc/%d/stat" % int(pid)) as f:
                s = f.read()
            return s[s.rindex(")") + 2:].split()[0] != "Z"      # a zombie has ended
        except (OSError, ValueError, IndexError):
            try:
                os.kill(int(pid), 0)
                return True
            except OSError:
                return False
    if pgid:
        try:
            import stop_confirm as SC
            return bool([p for p, c in SC.group_procs(int(pgid)) if "pi_monitor.py" not in c])
        except Exception:  # noqa: BLE001
            return False
    return True


def watch(out, pgid=None, pid=None, newest=None, every_s=EVERY_S, max_s=None, sampler=None):
    """Rows into out/pi_monitor.csv until the run ends. newest: a glob; the newest folder made after this started is
    the run folder (home free roam makes its own folder under logs/)."""
    t0 = time.time()
    if not pgid and not pid and max_s is None:
        raise SystemExit("pi_monitor watch needs the run to end with: --pgid, --pid or --max-s")
    if newest:
        while group_alive(pgid, pid) and (max_s is None or time.time() - t0 < max_s):
            c = [d for d in glob.glob(newest) if os.path.isdir(d) and os.path.getmtime(d) >= t0 - 5]
            if c:
                out = max(c, key=os.path.getmtime)
                break
            time.sleep(1.0)
        else:
            return None
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, NAME)
    smp = sampler or Sampler(out)
    new = not os.path.exists(path)
    with open(path, "a", newline="", buffering=1) as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        if new:
            w.writeheader()
        try:
            while group_alive(pgid, pid) and (max_s is None or time.time() - t0 < max_s):
                time.sleep(every_s)
                try:
                    w.writerow(smp.row())
                except Exception:  # noqa: BLE001  (a missed row, never a stopped monitor)
                    pass
        except KeyboardInterrupt:
            pass
    try:
        report(out)
    except Exception:  # noqa: BLE001
        pass
    return path


def on():
    """MIRLT_PI_MONITOR=0 switches the monitor off (decided automatically: on)."""
    return os.environ.get("MIRLT_PI_MONITOR", "1") != "0"


def start_for(out, pgid=None, pid=None, newest=None):
    """Mission Control: the monitor in its own process (its own session: never part of the run's Ctrl+C), low
    priority. Returns the Popen or None."""
    if not on():
        return None
    cmd = [sys.executable, os.path.abspath(__file__), "watch", "--out", out]
    if pgid:
        cmd += ["--pgid", str(pgid)]
    if pid:
        cmd += ["--pid", str(pid)]
    if newest:
        cmd += ["--newest", newest]
    if shutil.which("nice"):
        cmd = ["nice", "-n", "10"] + cmd
    try:
        os.makedirs(out, exist_ok=True)
        return subprocess.Popen(cmd, stdout=open(os.path.join(out, "pi_monitor.log"), "a"), stderr=subprocess.STDOUT,
                                start_new_session=True, cwd=os.path.dirname(TOOLS))
    except OSError:
        return None


# ------------------------------------------------------------------------------------------------ health (warnings)
def rows_of(path):
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def p95(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return xs[min(len(xs) - 1, int(0.95 * (len(xs) - 1) + 0.5))]


def level(v, red, amber):
    if v is None:
        return "none"
    if red is not None and v >= red:
        return "red"
    if amber is not None and v > amber:
        return "amber"
    return "ok"


def health(rows, floor_gb=None):
    """The newest row against the limits: {"items": [{key, label, value, unit, level, why}], "level", "words"}."""
    if not rows:
        return {"items": [], "level": "none", "words": "No readings yet."}
    last = rows[-1]
    hist = rows[:-1] if len(rows) > WARMUP_ROWS else []
    floor_gb = card_floor_gb() if floor_gb is None else floor_gb
    items = []

    def add(key, label, unit, v, red, amber, why):
        items.append({"key": key, "label": label, "unit": unit, "value": v, "level": level(v, red, amber), "why": why})
    cores = [_f(last.get("core%d" % k)) for k in range(4)]
    prev = rows[-2] if len(rows) > 1 else {}
    for k, v in enumerate(cores):
        if v is None:
            continue
        full = v >= CORE_FULL_PCT and (_f(prev.get("core%d" % k)) or 0) >= CORE_FULL_PCT
        a = p95([_f(r.get("core%d" % k)) for r in hist]) if hist else None
        items.append({"key": "core%d" % k, "label": "core %d" % k, "unit": "%", "value": v,
                      "level": "red" if full else level(v, None, a),
                      "why": "full two readings running" if full else
                      ("over this run's own 95th percentile (%.0f %%)" % a if a is not None and v > a else "")})
    t = _f(last.get("temp_c"))
    add("temp_c", "temperature", "C", t, TEMP_LIMIT_C, None, "the Pi slows its clock from %.0f C" % TEMP_LIMIT_C)
    tn = last.get("throttle_now") or ""
    items.append({"key": "throttle", "label": "throttling", "unit": "", "value": tn or ("none" if last.get("throttled")
                                                                                       else "not available here"),
                  "level": "red" if tn else ("ok" if last.get("throttled") else "none"),
                  "why": ("the Pi's own flags now: " + tn) if tn else (
                      "since boot: " + last["throttle_since_boot"] if last.get("throttle_since_boot") else "")})
    for cam in ("overhead", "side"):
        v = _f(last.get("lag_%s_s" % cam))
        a = p95([_f(r.get("lag_%s_s" % cam)) for r in hist]) if hist else None
        add("lag_" + cam, "%s picture lag" % cam, "s", v, FIX_MAX_AGE_S, a,
            "a robot throws away a fix over %.0f s old" % FIX_MAX_AGE_S)
    v = _f(last.get("loop_work_ms_p95"))
    a = p95([_f(r.get("loop_work_ms_p95")) for r in hist]) if hist else None
    add("loop", "planner loop (95 %)", "ms", v, LOOP_TARGET_MS, a,
        "the planner aims at one pass every %.0f ms" % LOOP_TARGET_MS)
    sh, cap = _f(last.get("det_share_pct")), _f(last.get("det_cap_pct"))
    add("detector", "detector share of its core", "%", sh, None, cap,
        "its cap is %.0f %% of one core" % cap if cap is not None else "the detector is not running")
    ma, mt = _f(last.get("mem_avail_mb")), _f(last.get("mem_total_mb"))
    if ma is not None and mt:
        items.append({"key": "mem", "label": "memory free", "unit": "MB", "value": ma,
                      "level": "red" if ma < MEM_RED_FRAC * mt else "ok", "why": "of %.0f MB" % mt})
    d = _f(last.get("disk_free_gb"))
    if d is not None:
        items.append({"key": "disk", "label": "card free", "unit": "GB", "value": d,
                      "level": "red" if d < floor_gb else ("amber" if d < DISK_AMBER_X * floor_gb else "ok"),
                      "why": "runs stop under %.2f GB (diskguard)" % floor_gb})
    lv = "red" if any(i["level"] == "red" for i in items) else "amber" if any(i["level"] == "amber" for i in items) \
        else "ok"
    bad = [i for i in items if i["level"] in ("red", "amber")]
    words = "All inside their limits." if not bad else "; ".join(
        "%s %s%s (%s)" % (i["label"], _fmt(i["value"]), i["unit"], i["why"]) for i in bad)
    return {"items": items, "level": lv, "words": words, "t": _f(last.get("t")), "rows": len(rows)}


def _fmt(v):
    return ("%.2f" % v if abs(v) < 10 else "%.0f" % v) if isinstance(v, float) else str(v)


class Idle:
    """Mission Control between runs: the same readings (no run folder, no feed), each a fresh row on demand."""

    def __init__(self, folder):
        self.s = Sampler(folder)
        self.rows = []

    def now(self):
        r = self.s.row()
        self.rows = (self.rows + [{k: ("" if v is None else v) for k, v in r.items()}])[-120:]
        return self.rows


# ------------------------------------------------------------------------------------------------ the run report
def summary(rows):
    def col(k):
        return [_f(r.get(k)) for r in rows if _f(r.get(k)) is not None]

    def stats(k):
        xs = col(k)
        return {"median": round(statistics.median(xs), 2) if xs else None, "p95": p95(xs), "max": max(xs) if xs
                else None, "n": len(xs)}
    return {k: stats(k) for k in ("core0", "core1", "core2", "core3", "temp_c", "lag_overhead_s", "lag_side_s",
                                  "loop_work_ms_p95", "loop_period_ms", "det_share_pct", "det_ms_p50")} | {
        "throttle_rows": sum(1 for r in rows if r.get("throttle_now")), "rows": len(rows),
        "det_calls": max(col("det_calls")) if col("det_calls") else 0}


def chart(rows, out_png, title="Pi health during this run"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = _f(rows[0]["t"])
    m = [(_f(r["t"]) - t0) / 60.0 for r in rows]
    fig, ax = plt.subplots(3, 1, figsize=(8, 6.6), sharex=True)
    cc = ["#2952CC", "#C2410C", "#7A3EC8", "#0E7C86"]
    for k in range(4):
        ys = [_f(r.get("core%d" % k)) for r in rows]
        if any(y is not None for y in ys):
            ax[0].plot(m, [y if y is not None else float("nan") for y in ys], lw=1, color=cc[k], label="core %d" % k)
    ds = [_f(r.get("det_share_pct")) for r in rows]
    if any(d is not None for d in ds):
        ax[0].plot(m, [d if d is not None else float("nan") for d in ds], lw=1.6, color="#111", ls="--",
                   label="detector's share of its core")
    ax[0].set_ylabel("busy %")
    ax[0].set_ylim(0, 105)
    ax[0].legend(fontsize=7, ncol=5, loc="upper left")
    ax[0].set_title(title, fontsize=11)
    for cam, col in (("overhead", "#2952CC"), ("side", "#C2410C")):
        ys = [_f(r.get("lag_%s_s" % cam)) for r in rows]
        if any(y is not None for y in ys):
            ax[1].plot(m, [y if y is not None else float("nan") for y in ys], lw=1, color=col, label="%s camera" % cam)
    ax[1].axhline(FIX_MAX_AGE_S, color="#B42318", lw=0.8, ls=":")
    ax[1].text(0, FIX_MAX_AGE_S, " a robot drops fixes older than this", fontsize=7, color="#B42318", va="bottom")
    ax[1].set_ylabel("picture lag (s)")
    if ax[1].get_legend_handles_labels()[0]:
        ax[1].legend(fontsize=7, loc="upper right")
    else:
        ax[1].text(0.5, 0.5, "no picture lag measured (no recorder answering: a practice run)", transform=ax[1].transAxes,
                   ha="center", fontsize=8, color="#555")
    lw = [_f(r.get("loop_work_ms_p95")) for r in rows]
    ax[2].plot(m, [y if y is not None else float("nan") for y in lw], lw=1, color="#0E7C86",
               label="planner loop, 95 % of passes (ms)")
    ax[2].axhline(LOOP_TARGET_MS, color="#0E7C86", lw=0.8, ls=":")
    ax[2].set_ylim(bottom=0)
    tt = [_f(r.get("temp_c")) for r in rows]
    ax2 = ax[2].twinx()
    if any(y is not None for y in tt):
        ax2.plot(m, [y if y is not None else float("nan") for y in tt], lw=1.2, color="#B54708", label="temperature (C)")
        ax2.axhline(TEMP_LIMIT_C, color="#B54708", lw=0.8, ls=":")
        ax2.set_ylabel("C")
    else:
        ax2.set_yticks([])
        ax[2].text(0.99, 0.9, "no temperature sensor here (not a Pi)", transform=ax[2].transAxes, ha="right", fontsize=7,
                   color="#555")
    ax[2].set_ylabel("ms")
    ax[2].set_xlabel("minutes into the run")
    h1, l1 = ax[2].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, fontsize=7, loc="upper left")
    for i, r in enumerate(rows):
        if r.get("throttle_now"):
            for a in ax:
                a.axvspan(m[i] - 1 / 60.0, m[i] + 1 / 60.0, color="#B42318", alpha=0.15, lw=0)
    fig.text(0.01, 0.005, "Measured on the Pi every 2 s by tools/pi/pi_monitor.py: /proc/stat, the thermal sensor, "
             "vcgencmd, the recorder's picture times and the planner's own loop and detector numbers. Source: "
             "pi_monitor.csv. Dotted lines: the documented limits.", fontsize=6.5, color="#555", wrap=True)
    fig.tight_layout(rect=(0, 0.035, 1, 1))
    fig.savefig(out_png, dpi=100)
    plt.close(fig)
    return out_png


def report(folder):
    """The run report's section: pi_monitor.png and pi_monitor.md in the run folder. Returns the markdown lines."""
    rows = rows_of(os.path.join(folder, NAME))
    if not rows:
        return []
    s = summary(rows)
    h = health(rows)
    L = ["## Pi health (pi_monitor.csv)", ""]
    try:
        chart(rows, os.path.join(folder, "pi_monitor.png"))
        L += ["![Pi health](pi_monitor.png)", "",
              "Each line is one measure of how hard the Pi worked during this run; the dotted lines are the limits "
              "where it starts to hurt the robots.", ""]
    except Exception as e:  # noqa: BLE001
        L += ["(chart not made: %s)" % e, ""]

    def g(k, f="median"):
        v = s[k][f]
        return "-" if v is None else ("%.2f" % v if abs(v) < 10 else "%.0f" % v)
    L += ["| | median | 95 % | max |", "|---|---|---|---|"]
    for k, lab in (("core0", "core 0 busy % (recorder)"), ("core1", "core 1 busy % (planner + detector)"),
                   ("core2", "core 2 busy % (bridge)"), ("core3", "core 3 busy % (bridge)"),
                   ("temp_c", "temperature C"), ("lag_overhead_s", "overhead picture lag s"),
                   ("lag_side_s", "side picture lag s"), ("loop_work_ms_p95", "planner loop ms (95 % of passes)"),
                   ("det_share_pct", "detector share of its core %"), ("det_ms_p50", "detector ms a call")):
        if s[k]["n"]:
            L.append("| %s | %s | %s | %s |" % (lab, g(k), g(k, "p95"), g(k, "max")))
    L += ["", "Rows: %d (every 2 s). Throttle flags set in %d rows. Detector calls: %d. At the end: %s" % (
        s["rows"], s["throttle_rows"], s["det_calls"], h["words"]), "",
          "<sub>Core numbers follow core layout B (recorder 0, planner 1, bridge 2-3). The 95 % column: 95 of 100 "
          "readings were at or under it.</sub>", ""]
    with open(os.path.join(folder, "pi_monitor.md"), "w") as f:
        f.write("\n".join(L))
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("watch")
    w.add_argument("--out", default="")
    w.add_argument("--pgid", type=int, default=0)
    w.add_argument("--pid", type=int, default=0)
    w.add_argument("--newest", default="")
    w.add_argument("--every", type=float, default=EVERY_S)
    w.add_argument("--max-s", type=float, default=None)
    r = sub.add_parser("report")
    r.add_argument("folder")
    sub.add_parser("now")
    a = ap.parse_args(argv)
    if a.cmd == "watch":
        p = watch(a.out, a.pgid or None, a.pid or None, a.newest or None, a.every, a.max_s)
        print("pi_monitor: %s" % p, flush=True)
    elif a.cmd == "report":
        print("\n".join(report(a.folder)) or "no pi_monitor.csv in %s" % a.folder)
    else:
        idle = Idle(os.getcwd())
        time.sleep(1.0)
        rows = idle.now()
        for k, v in rows[-1].items():
            print("%-20s %s" % (k, v))
        print(health(rows)["words"])


if __name__ == "__main__":
    main()
