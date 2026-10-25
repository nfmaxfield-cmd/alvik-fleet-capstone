"""The Pi's load, temperature and throttling, every few seconds, into the session folder (2026-10-25 evening,
MIRLT). The 15:45 session logged none of these, so whether the camera slowed for the Pi's sake (heat, a weak supply)
or the runner's (one Python process) could only be inferred. With the tracker in its own process, this shows whether
the split of cores is right: each core's load, and each of our processes' share.

cpu.csv, one row every EVERY_S:
  t, clock                       when (Unix seconds, local time)
  core0..coreN                   each core's busy share since the last row (0-100 %, /proc/stat)
  load1                          the 1-minute load average
  temp_c                         the SoC temperature (/sys/class/thermal/thermal_zone0/temp; vcgencmd measure_temp)
  throttled                      the firmware's throttle flags, hex (vcgencmd get_throttled, or /sys .../get_throttled);
                                 bit 0 under-voltage now, 1 frequency capped now, 2 throttled now, 3 soft temperature
                                 limit now; bits 16-19 the same "has happened since boot"
  throttle_words                 those bits in words ("" when none)
  freq_mhz                       core 0's clock now
  mem_avail_mb                   memory available
  <name>_cpu, <name>_rss_mb, <name>_cores   per watched process: its CPU share of ONE core since the last row (100 =
                                 one core busy, 200 = two), its memory, the cores it may run on
Watched processes: given by pid (the runner, the tracker process), or found by a word in their command line (the
recorder: "camrec.py"). Missing tools or files leave their column empty; nothing here can stop a session.

    w = CpuLog(folder, procs={"runner": os.getpid(), "tracker": pid}, find={"recorder": "camrec.py"})
    w.start() ... w.stop()
"""
import csv
import os
import shutil
import subprocess
import threading
import time

EVERY_S = 2.0            # chosen ("every few seconds"): a row every 2 s, 1800 rows an hour. No DOE: the resolution of a
                         # record; the camera's own slow spells last 5 s and more (perf.csv)
VCGENCMD_EVERY_S = 10.0  # chosen: vcgencmd starts a program (~10 ms); the /sys files, when there, are read every row.
                         # No DOE

THROTTLE_BITS = {0: "under-voltage", 1: "frequency capped", 2: "throttled", 3: "soft temperature limit",
                 16: "under-voltage has happened", 17: "frequency capping has happened", 18: "throttling has happened",
                 19: "soft temperature limit has happened"}
THROTTLE_SYS = ("/sys/devices/platform/soc/soc:firmware/get_throttled",
                "/sys/devices/platform/soc/soc:firmware/raspberrypi-hwmon/get_throttled")


def read_stat(path="/proc/stat"):
    """{core index: (busy jiffies, total jiffies)} from /proc/stat."""
    out = {}
    try:
        with open(path) as f:
            for ln in f:
                if ln.startswith("cpu") and ln[3:4].isdigit():
                    p = ln.split()
                    v = [int(x) for x in p[1:]]
                    idle = v[3] + (v[4] if len(v) > 4 else 0)
                    out[int(p[0][3:])] = (sum(v) - idle, sum(v))
    except OSError:
        pass
    return out


def core_loads(a, b):
    """Busy % per core between two read_stat() results."""
    out = {}
    for k in b:
        if k in a:
            db, dt = b[k][0] - a[k][0], b[k][1] - a[k][1]
            out[k] = round(100.0 * db / dt, 1) if dt > 0 else 0.0
    return out


def proc_times(pid):
    """(utime + stime in seconds, rss MB) of a process from /proc/<pid>/stat, or None."""
    try:
        with open("/proc/%d/stat" % pid) as f:
            s = f.read()
        p = s[s.rindex(")") + 2:].split()
        tick = os.sysconf("SC_CLK_TCK")
        cpu = (int(p[11]) + int(p[12])) / tick
        rss = int(p[21]) * os.sysconf("SC_PAGE_SIZE") / 1e6
        return cpu, rss
    except (OSError, ValueError, IndexError):
        return None


def find_pid(word, me=None):
    """The first process whose command line contains `word` (not this one), or None."""
    me = os.getpid() if me is None else me
    try:
        for d in os.listdir("/proc"):
            if not d.isdigit() or int(d) == me:
                continue
            try:
                with open("/proc/%s/cmdline" % d, "rb") as f:
                    cl = f.read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if word in cl and "bash -c" not in cl and "tmux" not in cl:
                return int(d)
    except OSError:
        pass
    return None


def read_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def read_freq():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq") as f:
            return round(int(f.read().strip()) / 1000.0)
    except (OSError, ValueError):
        return None


def read_mem_avail():
    try:
        with open("/proc/meminfo") as f:
            for ln in f:
                if ln.startswith("MemAvailable:"):
                    return round(int(ln.split()[1]) / 1024.0)
    except (OSError, ValueError):
        pass
    return None


def parse_throttled(s):
    """'throttled=0x50005' (vcgencmd) or '50005' (the /sys file, hex) -> int, or None."""
    if s is None:
        return None
    s = s.strip()
    if "=" in s:
        s = s.split("=", 1)[1]
    try:
        return int(s, 16)
    except ValueError:
        return None


def throttle_words(v):
    if not v:
        return ""
    return "; ".join(w for b, w in sorted(THROTTLE_BITS.items()) if v & (1 << b))


def vcgencmd(*args, run=subprocess.run):
    if not shutil.which("vcgencmd") and run is subprocess.run:
        return None
    try:
        r = run(["vcgencmd"] + list(args), capture_output=True, text=True, timeout=3)
        return r.stdout.strip() or None
    except Exception:
        return None


class CpuLog(threading.Thread):
    def __init__(self, folder, procs=None, find=None, every_s=EVERY_S, log=None, run=subprocess.run):
        super().__init__(daemon=True, name="cpu-log")
        self.path = os.path.join(folder, "cpu.csv")
        self.procs = dict(procs or {})           # name -> pid
        self.find = dict(find or {})             # name -> word in its command line
        self.every = float(every_s)
        self.log = log or (lambda m: None)
        self.run_ = run
        self.stop_ev = threading.Event()
        self.rows = 0
        self.last = None                         # the latest row (dict), for the live page / tests
        self.max_temp = None
        self.throttled_seen = 0

    def add_proc(self, name, pid):
        self.procs[name] = pid

    def _throttled(self, now, cache):
        for p in THROTTLE_SYS:
            try:
                with open(p) as f:
                    return parse_throttled(f.read())
            except OSError:
                continue
        if now - cache.get("t", -1e9) >= VCGENCMD_EVERY_S:
            cache["t"] = now
            cache["v"] = parse_throttled(vcgencmd("get_throttled", run=self.run_))
        return cache.get("v")

    def _temp(self, now, cache):
        t = read_temp()
        if t is not None:
            return t
        if now - cache.get("t", -1e9) >= VCGENCMD_EVERY_S:
            cache["t"] = now
            s = vcgencmd("measure_temp", run=self.run_)           # temp=48.7'C
            try:
                cache["v"] = float(s.split("=", 1)[1].split("'")[0]) if s else None
            except (ValueError, IndexError):
                cache["v"] = None
        return cache.get("v")

    def run(self):
        try:
            stat0 = read_stat()
            cores = sorted(stat0)
            names = sorted(set(self.procs) | set(self.find))
            cols = (["t", "clock"] + ["core%d" % c for c in cores] +
                    ["load1", "temp_c", "throttled", "throttle_words", "freq_mhz", "mem_avail_mb"] +
                    [n + s for n in names for s in ("_cpu", "_rss_mb", "_cores")])
            new = not os.path.exists(self.path)
            f = open(self.path, "a", newline="")
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            if new:
                w.writeheader()
            prev_p, t_prev = {}, time.time()
            thr_c, tmp_c = {}, {}
            while not self.stop_ev.wait(self.every):
                now = time.time()
                stat1 = read_stat()
                row = dict(t="%.2f" % now, clock=time.strftime("%H:%M:%S", time.localtime(now)))
                for c, v in core_loads(stat0, stat1).items():
                    row["core%d" % c] = v
                stat0 = stat1
                try:
                    row["load1"] = round(os.getloadavg()[0], 2)
                except OSError:
                    pass
                tc = self._temp(now, tmp_c)
                row["temp_c"] = "" if tc is None else tc
                if tc is not None:
                    self.max_temp = tc if self.max_temp is None else max(self.max_temp, tc)
                th = self._throttled(now, thr_c)
                row["throttled"] = "" if th is None else "0x%x" % th
                row["throttle_words"] = throttle_words(th)
                if th:
                    self.throttled_seen |= th
                row["freq_mhz"] = read_freq() or ""
                row["mem_avail_mb"] = read_mem_avail() or ""
                dt = now - t_prev
                t_prev = now
                for n in names:
                    pid = self.procs.get(n)
                    if pid is None and n in self.find:
                        pid = find_pid(self.find[n])
                        if pid is not None:
                            self.procs[n] = pid
                    if pid is None:
                        continue
                    pt = proc_times(pid)
                    if pt is None:
                        if n in self.find:
                            self.procs.pop(n, None)      # ended: look for it again
                        continue
                    if n in prev_p and prev_p[n][0] == pid and dt > 0:
                        row[n + "_cpu"] = round(100.0 * (pt[0] - prev_p[n][1]) / dt, 1)
                    row[n + "_rss_mb"] = round(pt[1], 1)
                    try:
                        row[n + "_cores"] = " ".join(str(c) for c in sorted(os.sched_getaffinity(pid)))
                    except (OSError, AttributeError):
                        pass
                    prev_p[n] = (pid, pt[0])
                w.writerow(row)
                f.flush()
                self.rows += 1
                self.last = row
            f.close()
        except Exception as e:
            self.log("(cpu.csv stopped: %s)" % e)

    def stop(self):
        self.stop_ev.set()

    def summary(self):
        return dict(rows=self.rows, max_temp_c=self.max_temp, throttled_bits="0x%x" % self.throttled_seen,
                    throttled_words=throttle_words(self.throttled_seen))
