#!/usr/bin/env python3
"""Where the Pi's time goes (build e16, 11 Nov 2026; the school Pi's 1-minute load of 8-10 on 10 Nov, while the tracked
programs' CPU was the same as on a run with load 4.5-5).

    python3 tools/pi/pi_bench.py monitor --out DIR [--every 2] [--seconds 600]
    python3 tools/pi/pi_bench.py report DIR [--label "trimmed defaults"]      # DIR/report.md
    python3 tools/pi/pi_bench.py compare DIR1 DIR2 [DIR3] --out FILE.md       # side by side (+ FILE.png)

monitor, every --every seconds, from /proc and /sys only (nothing it reads can stop or slow a run):
  system.csv   t, load1, cores busy (sum, %), iowait (sum, %), tasks running (R) and waiting on a device (D) now (the
               load average counts BOTH: a D task is waiting for the card, the USB drive or a camera, using no CPU),
               clock now (MHz, core 0), temperature, throttle flags, memory available
  procs.csv    t, pid, name, cpu % of one core, threads, state; every process over 0.5 % (or one of ours)
  threads.csv  t, pid, tid, process, thread name, cpu % of one core, state; every thread over 1 %
  dstate.csv   t, pid, tid, process, thread name, wchan (what the kernel says it waits on): every task in D state
Each process is labelled with its role (recorder, tag bridge, run_plan, mission_run, Mission Control, simulator ...)
from its command line, so the report can say which work exists in a real run and which only in the bench.

report: per role mean and 95th percentile CPU, the busiest threads, the load against busy cores (what the load is made
of: running, waiting for a core, waiting on a device), the clock's lowest and how long it sat under 1500 MHz, the hottest,
the throttle flags, the tag bridge's detection time per picture (bridge/detect_times.csv) and run_plan's plan times
(plan_N.json planning_time_s, the first plan left out). Lean: numbers, then one line of what they say.
"""
import argparse
import csv
import glob
import json
import os
import re
import statistics as st
import sys
import time

VERSION = "pi-bench-2026-11-12e17"   # e17: pictures the real bridge handled a second, picture-to-result lag (detect_times t_done), the practice run's watchdog stops and message gaps, the core layout in the setup line; bench layouts A / B / C (run-fleet-pi.sh). e16: first version (per-process and per-thread CPU, R / D tasks, clock, heat)

ROLES = (("camrec.py", "recorder"), ("tag_bridge.py", "tag bridge"), ("cam_bridge.py", "camera bridge"),
         ("run_plan.py", "run_plan"), ("rp_shim.py", "run_plan (practice)"), ("mission_run.py", "mission_run"),
         ("mission_control.py", "Mission Control"), ("freeroam.py", "free roam"), ("calib_run.py", "calibration"),
         ("site_fit.py", "camera refit (background)"), ("fleet_sim.py", "simulator (not in a real run)"),
         ("sim_shim", "simulator (not in a real run)"), ("sim_cameras", "simulator (not in a real run)"),
         ("pi_bench.py", "this monitor"), ("upload_session.py", "upload"), ("tmux", "tmux"),
         ("sshd", "ssh"), ("python3", "other python"))
SIM_ROLES = ("simulator (not in a real run)", "this monitor", "tag bridge (practice: simulated pictures)", "py-spy",
             "claude")
THROTTLE_SYS = ("/sys/devices/platform/soc/soc:firmware/get_throttled",
                "/sys/devices/platform/soc/soc:firmware/raspberrypi-hwmon/get_throttled")
TICK = os.sysconf("SC_CLK_TCK")


def role_of(cmd, name):
    if "tag_bridge.py" in cmd and "--observe" in cmd:
        return "tag bridge (real camera)"
    if "tag_bridge.py" in cmd and "-dry" in cmd:
        return "tag bridge (practice: simulated pictures)"
    for word, role in ROLES:
        if word in cmd:
            return role
    return name


def _read(p):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return ""


def _stat_fields(s):
    """(comm, state, utime + stime ticks, threads) from a /proc/.../stat line."""
    try:
        i, j = s.index("("), s.rindex(")")
        rest = s[j + 2:].split()
        return s[i + 1:j], rest[0], int(rest[11]) + int(rest[12]), int(rest[17])
    except (ValueError, IndexError):
        return None


def cpu_totals():
    """(busy ticks, iowait ticks, total ticks, per-core count) from /proc/stat's first line."""
    for ln in _read("/proc/stat").splitlines():
        if ln.startswith("cpu "):
            v = [int(x) for x in ln.split()[1:]]
            idle, iow = v[3], (v[4] if len(v) > 4 else 0)
            n = sum(1 for l2 in _read("/proc/stat").splitlines() if l2.startswith("cpu") and l2[3:4].isdigit())
            return sum(v) - idle - iow, iow, sum(v), max(1, n)
    return 0, 0, 1, 1


def throttled():
    for p in THROTTLE_SYS:
        s = _read(p).strip()
        if s:
            try:
                return int(s, 16)
            except ValueError:
                pass
    return None


def snapshot():
    """{(pid, tid): (comm, state, ticks)} for every task, {pid: (comm, cmd, ticks, threads, state)} per process."""
    tasks, procs = {}, {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        pid = int(d)
        f = _stat_fields(_read("/proc/%d/stat" % pid))
        if f is None:
            continue
        cmd = _read("/proc/%d/cmdline" % pid).replace("\0", " ").strip()
        procs[pid] = (f[0], cmd, f[2], f[3], f[1])
        try:
            tids = os.listdir("/proc/%d/task" % pid)
        except OSError:
            continue
        for t in tids:
            g = _stat_fields(_read("/proc/%d/task/%s/stat" % (pid, t)))
            if g is not None:
                tasks[(pid, int(t))] = (g[0], g[1], g[2])
    return tasks, procs


def monitor(out, every=2.0, seconds=0.0, stop_file=None):
    os.makedirs(out, exist_ok=True)
    files = {}

    def w(name, cols):
        if name not in files:
            f = open(os.path.join(out, name), "a", newline="")
            wr = csv.writer(f)
            if f.tell() == 0:
                wr.writerow(cols)
            files[name] = (f, wr)
        return files[name][1]
    sw = w("system.csv", ["t", "load1", "busy_pct", "iowait_pct", "n_running", "n_dstate", "freq_mhz", "temp_c",
                          "throttled", "mem_avail_mb", "cores"])
    pw = w("procs.csv", ["t", "pid", "role", "name", "cpu_pct", "threads", "state", "cmd"])
    tw = w("threads.csv", ["t", "pid", "tid", "role", "thread", "cpu_pct", "state"])
    dw = w("dstate.csv", ["t", "pid", "tid", "role", "thread", "wchan"])
    t0 = time.time()
    prev_tasks, prev_procs = snapshot()
    prev_cpu = cpu_totals()
    prev_t = time.time()
    me = os.getpid()
    while True:
        time.sleep(every)
        now = time.time()
        tasks, procs = snapshot()
        cpu = cpu_totals()
        dt = max(1e-3, now - prev_t)
        dtot = max(1, cpu[2] - prev_cpu[2])
        busy = 100.0 * cpu[3] * (cpu[0] - prev_cpu[0]) / dtot
        iow = 100.0 * cpu[3] * (cpu[1] - prev_cpu[1]) / dtot
        n_r = sum(1 for v in tasks.values() if v[1] == "R")
        n_d = sum(1 for v in tasks.values() if v[1] == "D")
        try:
            load1 = os.getloadavg()[0]
        except OSError:
            load1 = ""
        fr = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").strip()
        tc = _read("/sys/class/thermal/thermal_zone0/temp").strip()
        mem = ""
        for ln in _read("/proc/meminfo").splitlines():
            if ln.startswith("MemAvailable:"):
                mem = round(int(ln.split()[1]) / 1024.0)
        th = throttled()
        sw.writerow(["%.2f" % now, load1, round(busy, 1), round(iow, 1), n_r, n_d,
                     round(int(fr) / 1000.0) if fr.isdigit() else "", round(int(tc) / 1000.0, 1) if tc.lstrip(
                         "-").isdigit() else "", "" if th is None else "0x%x" % th, mem, cpu[3]])
        for pid, (comm, cmd, ticks, nth, state) in procs.items():
            if pid not in prev_procs or prev_procs[pid][0] != comm:
                continue
            c = 100.0 * (ticks - prev_procs[pid][2]) / TICK / dt
            role = role_of(cmd, comm) if pid != me else "this monitor"
            if c >= 0.5 or role in dict(ROLES).values():
                pw.writerow(["%.2f" % now, pid, role, comm, round(c, 1), nth, state,
                             cmd[:120] if role in ("other python", comm) else ""])
        roles = {pid: (role_of(v[1], v[0]) if pid != me else "this monitor") for pid, v in procs.items()}
        for (pid, tid), (comm, state, ticks) in tasks.items():
            p0 = prev_tasks.get((pid, tid))
            if p0 is not None and p0[0] == comm:
                c = 100.0 * (ticks - p0[2]) / TICK / dt
                if c >= 1.0:
                    tw.writerow(["%.2f" % now, pid, tid, roles.get(pid, "?"), comm, round(c, 1), state])
            if state == "D":
                dw.writerow(["%.2f" % now, pid, tid, roles.get(pid, "?"), comm,
                             _read("/proc/%d/task/%d/wchan" % (pid, tid)).strip()])
        for f, _ in files.values():
            f.flush()
        prev_tasks, prev_procs, prev_cpu, prev_t = tasks, procs, cpu, now
        if (seconds and now - t0 >= seconds) or (stop_file and os.path.exists(stop_file)):
            break
    for f, _ in files.values():
        f.close()


# ----------------------------------------------------------------------------------------------------- the report
def _rows(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def pct(v, q):
    v = sorted(x for x in v if x is not None)
    if not v:
        return None
    return v[min(len(v) - 1, int(round(q / 100.0 * (len(v) - 1))))]


def summarise(d):
    """The numbers a bench folder gives (dict)."""
    s = _rows(os.path.join(d, "system.csv"))
    out = {"folder": d, "samples": len(s)}
    if not s:
        return out
    col = lambda k: [_f(r.get(k)) for r in s]           # noqa: E731
    out.update(load1_med=pct(col("load1"), 50), load1_p95=pct(col("load1"), 95),
               busy_med=pct(col("busy_pct"), 50), iowait_med=pct(col("iowait_pct"), 50),
               r_med=pct(col("n_running"), 50), d_med=pct(col("n_dstate"), 50), d_max=pct(col("n_dstate"), 100),
               freq_min=pct(col("freq_mhz"), 0), temp_max=pct(col("temp_c"), 100), temp_med=pct(col("temp_c"), 50),
               mem_min=pct(col("mem_avail_mb"), 0), cores=_f(s[-1].get("cores")))
    fr = [x for x in col("freq_mhz") if x is not None]
    out["freq_low_share"] = round(sum(1 for x in fr if x < 1500) / len(fr), 3) if fr else None
    th = 0
    for r in s:
        try:
            th |= int(r.get("throttled") or "0", 16)
        except ValueError:
            pass
    out["throttled_bits"] = "0x%x" % th
    p = _rows(os.path.join(d, "procs.csv"))
    ts = sorted(set(r["t"] for r in p))
    by = {}
    for r in p:
        by.setdefault(r["role"], {}).setdefault(r["t"], 0.0)
        by[r["role"]][r["t"]] += _f(r["cpu_pct"]) or 0.0
    out["roles"] = {k: dict(mean=round(sum(v.get(t, 0.0) for t in ts) / max(1, len(ts)), 1),
                            p95=round(pct([v.get(t, 0.0) for t in ts], 95) or 0.0, 1)) for k, v in by.items()}
    out["real_run_cpu_mean"] = round(sum(v["mean"] for k, v in out["roles"].items() if k not in SIM_ROLES), 1)
    t_ = _rows(os.path.join(d, "threads.csv"))
    tt = {}
    for r in t_:
        k = (r["role"], r["thread"])
        tt.setdefault(k, 0.0)
        tt[k] += _f(r["cpu_pct"]) or 0.0
    n = max(1, len(set(r["t"] for r in t_)))
    out["threads"] = sorted(([k[0], k[1], round(v / n, 1)] for k, v in tt.items()), key=lambda q: -q[2])[:15]
    dd = _rows(os.path.join(d, "dstate.csv"))
    dc = {}
    for r in dd:
        dc[(r["role"], r["thread"], r["wchan"])] = dc.get((r["role"], r["thread"], r["wchan"]), 0) + 1
    out["dstate_top"] = sorted(([k[0], k[1], k[2], v] for k, v in dc.items()), key=lambda q: -q[3])[:8]
    det, lag, rate = [], [], []
    for f in glob.glob(os.path.join(d, "**", "detect_times.csv"), recursive=True):
        if "practice_run" in f or "-dry" in f:
            continue                     # the practice's simulated pictures: not the real camera's detection
        rows = _rows(f)
        det += [_f(r.get("ms")) for r in rows]
        # (e17) pictures handled a second per camera, and the picture-to-result lag (t_done, e17's bridge)
        lag += [(_f(r.get("t_done")) or 0.0) - (_f(r.get("t")) or 0.0) for r in rows if _f(r.get("t_done"))]
        by = {}
        for r in rows:
            if _f(r.get("t")) is not None:
                by.setdefault(r.get("cam"), []).append(_f(r.get("t")))
        for ts in by.values():
            if len(ts) > 10 and max(ts) > min(ts):
                rate.append(len(ts) / (max(ts) - min(ts)))
    if lag:
        out.update(lag_s_med=pct(lag, 50), lag_s_p90=pct(lag, 90))
    if rate:
        out["pictures_per_s"] = round(sum(rate) / len(rate), 2)
    wd, gaps = _watchdog(d)
    out.update(watchdog_stops=wd, msg_gaps_over_wd=gaps)
    det = [x for x in det if x is not None]
    if det:
        out.update(detect_ms_med=pct(det, 50), detect_ms_p95=pct(det, 95), detect_n=len(det))
    plans = []
    for f in glob.glob(os.path.join(d, "**", "plan_*.json"), recursive=True):
        if f.endswith("plan_given.json") or f.endswith("plan_0.json"):
            continue                     # the run's first plan (made before the run) is not a re-plan
        try:
            with open(f) as fh:
                v = (json.load(fh).get("summary") or {}).get("planning_time_s")
            if v is not None:
                plans.append(float(v))
        except (OSError, ValueError):
            pass
    if plans:
        out.update(plan_s_med=pct(plans, 50), plan_s_max=max(plans), plan_n=len(plans))
    info = os.path.join(d, "bench.json")
    if os.path.exists(info):
        with open(info) as f:
            out["bench"] = json.load(f)
    return out


WD_S = 1.5    # the robots' watchdog (follow_wd_ms 1500): a gap this long in what a robot hears stops it


def _watchdog(d):
    """(e17) In the bench's practice run: the watchdog stops its robots reported, and the gaps over WD_S between
    lines the planner sent each robot (messages / sent logs where the run keeps them); None where nothing was found."""
    n, gaps, seen = 0, 0, False
    for f in glob.glob(os.path.join(d, "**", "events*.csv"), recursive=True) + \
            glob.glob(os.path.join(d, "**", "coordinator.jsonl"), recursive=True):
        seen = True
        try:
            with open(f, errors="replace") as fh:
                n += sum(1 for line in fh if "watchdog" in line.lower())
        except OSError:
            pass
    for f in glob.glob(os.path.join(d, "**", "sent.csv"), recursive=True):
        by = {}
        for r in _rows(f):
            if _f(r.get("t")) is not None:
                by.setdefault(r.get("robot"), []).append(_f(r.get("t")))
        for ts in by.values():
            ts.sort()
            gaps += sum(1 for a, b in zip(ts, ts[1:]) if b - a > WD_S)
    return (n if seen else None), (gaps if seen else None)


def fmt(v, nd=1):
    return "-" if v is None else (("%." + str(nd) + "f") % v)


def report(d, label=""):
    s = summarise(d)
    lines = ["# Pi bench: %s" % (label or os.path.basename(d.rstrip("/"))), ""]
    b = s.get("bench") or {}
    if b:
        lines.append("Setup: %s" % "; ".join("%s %s" % (k, v) for k, v in b.items()))
        lines.append("")
    lines += ["| What | Value |", "|---|---|",
              "| 1-min load, median (95th pct) | %s (%s) |" % (fmt(s.get("load1_med"), 2), fmt(s.get("load1_p95"), 2)),
              "| Cores busy, median (sum, %%; %d cores = %d %%) | %s |" % (s.get("cores") or 0, 100 * (s.get("cores") or 0),
                                                                         fmt(s.get("busy_med"))),
              "| Waiting on a device (iowait, sum %%), median | %s |" % fmt(s.get("iowait_med")),
              "| Tasks running / waiting on a device (D), median (D max) | %s / %s (%s) |" % (
                  fmt(s.get("r_med"), 0), fmt(s.get("d_med"), 0), fmt(s.get("d_max"), 0)),
              "| Clock, lowest MHz (share of samples under 1500) | %s (%s) |" % (
                  fmt(s.get("freq_min"), 0), fmt(100 * s["freq_low_share"], 0) + " %" if s.get(
                      "freq_low_share") is not None else "-"),
              "| Temperature, median (max) C | %s (%s) |" % (fmt(s.get("temp_med")), fmt(s.get("temp_max"))),
              "| Throttle flags seen | %s |" % s.get("throttled_bits", "-"),
              "| Memory available, lowest MB | %s |" % fmt(s.get("mem_min"), 0),
              "| Tag detection per picture, median (95th pct) ms | %s (%s), %s pictures |" % (
                  fmt(s.get("detect_ms_med")), fmt(s.get("detect_ms_p95")), s.get("detect_n", 0)),
              "| Pictures the real camera's bridge handled a second (per camera) | %s |" % fmt(s.get("pictures_per_s")),
              "| Picture to result (fix delay at the bridge), median (90th pct) s | %s (%s) |" % (
                  fmt(s.get("lag_s_med"), 2), fmt(s.get("lag_s_p90"), 2)),
              "| Practice run: watchdog stops (message gaps over %.1f s) | %s (%s) |" % (
                  WD_S, s.get("watchdog_stops", "-"), s.get("msg_gaps_over_wd", "-")),
              "| run_plan re-plan time, median (max) s | %s (%s), %s re-plans |" % (
                  fmt(s.get("plan_s_med"), 2), fmt(s.get("plan_s_max"), 2), s.get("plan_n", 0)),
              "", "The practice's simulator and its simulated-picture tag bridge are measured but left out of the "
              "total: a real run has neither.", "", "CPU per program (%% of one core; mean, 95th pct). Total of what a real run has: **%s %%**." % fmt(
                  s.get("real_run_cpu_mean")), "", "| Program | Mean | 95th pct |", "|---|---|---|"]
    for k, v in sorted((s.get("roles") or {}).items(), key=lambda q: -q[1]["mean"]):
        if v["mean"] >= 0.5:
            lines.append("| %s | %s | %s |" % (k, fmt(v["mean"]), fmt(v["p95"])))
    lines += ["", "Busiest threads (mean %% of one core):", "", "| Program | Thread | Mean |", "|---|---|---|"]
    lines += ["| %s | %s | %s |" % tuple(t) for t in s.get("threads") or []]
    if s.get("dstate_top"):
        lines += ["", "Tasks seen waiting on a device (D), by what the kernel says they wait on (samples):", "",
                  "| Program | Thread | Waits on | Samples |", "|---|---|---|---|"]
        lines += ["| %s | %s | %s | %d |" % tuple(t) for t in s["dstate_top"]]
    lines += ["", "Source: %s (system.csv, procs.csv, threads.csv, dstate.csv, every %s s; detect_times.csv; "
              "messages.csv)." % (d, "2"), ""]
    with open(os.path.join(d, "report.md"), "w") as f:
        f.write("\n".join(lines))
    with open(os.path.join(d, "summary.json"), "w") as f:
        json.dump(s, f, indent=1, default=str)
    return "\n".join(lines)


def compare(dirs, out, labels=None):
    ss = [summarise(d) for d in dirs]
    labels = labels or [os.path.basename(d.rstrip("/")) for d in dirs]
    rows = [("1-min load, median", "load1_med", 2), ("Cores busy, median (sum %)", "busy_med", 0),
            ("Tasks waiting on a device (D), median", "d_med", 0), ("Clock lowest MHz", "freq_min", 0),
            ("Temperature max C", "temp_max", 1), ("Detection ms, median", "detect_ms_med", 1),
            ("Re-plan time s, median", "plan_s_med", 2), ("Real-run programs' CPU (sum %)", "real_run_cpu_mean", 0),
            ("Pictures handled a second (per camera)", "pictures_per_s", 1), ("Fix delay at the bridge s, median",
                                                                               "lag_s_med", 2),
            ("Fix delay s, 90th pct", "lag_s_p90", 2), ("Watchdog stops (practice)", "watchdog_stops", 0)]
    lines = ["| What | " + " | ".join(labels) + " |", "|---" * (len(labels) + 1) + "|"]
    for name, k, nd in rows:
        lines.append("| %s | %s |" % (name, " | ".join(fmt(s.get(k), nd) for s in ss)))
    roles = sorted(set(k for s in ss for k in (s.get("roles") or {})))
    for r in roles:
        vals = [(s.get("roles") or {}).get(r, {}).get("mean") for s in ss]
        if max((v or 0) for v in vals) >= 1.0:
            lines.append("| CPU: %s | %s |" % (r, " | ".join(fmt(v) for v in vals)))
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("monitor")
    m.add_argument("--out", required=True)
    m.add_argument("--every", type=float, default=2.0)
    m.add_argument("--seconds", type=float, default=0.0)
    m.add_argument("--stop-file", default="")
    r = sub.add_parser("report")
    r.add_argument("dir")
    r.add_argument("--label", default="")
    c = sub.add_parser("compare")
    c.add_argument("dirs", nargs="+")
    c.add_argument("--out", required=True)
    c.add_argument("--labels", default="")
    a = ap.parse_args(argv)
    if a.cmd == "monitor":
        monitor(a.out, a.every, a.seconds, a.stop_file or None)
    elif a.cmd == "report":
        print(report(a.dir, a.label))
    else:
        print(compare(a.dirs, a.out, [x for x in a.labels.split(",") if x] or None))
    return 0


if __name__ == "__main__":
    sys.exit(main())
