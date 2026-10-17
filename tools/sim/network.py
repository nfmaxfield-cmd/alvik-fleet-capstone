"""WiFi and clocks: turns what the robots sent (stamped with robot time) into
what the laptop console received and saved.

  - every line a robot sends becomes a UDP packet "name|seq|line";
  - it reaches the laptop after a random delay (packets stay in order, as
    they do through one access point), and RAW rows are sometimes lost;
  - the console asks each robot for its clock every 10 s (bursts of 3); the
    robot answers the next time its loop reads WiFi, which can be late while
    it is busy (inside a library move, a sync flash, the PARAM burst).

The packets are then fed, in arrival order, to the REAL console code
(tools/alvik_console.py, class Session) with its clock replaced by the
simulated arrival time. So the CSV files are written by the same code that
writes them in a real session, column for column.
"""
import bisect
import contextlib
import datetime as dt
import io
import json
import math
import os
import platform
import sys

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import alvik_console  # noqa: E402

CLOCK_EVERY_S = alvik_console.CLOCK_EVERY_S


class _FakeTime:
    """Stands in for the time module inside alvik_console while packets are replayed."""

    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now


def _poll_delay_ms(robot, arrive_ms, rng, slow_prob, slow_ms):
    """When the robot reads a command that arrived at robot time arrive_ms."""
    busy = robot._busy_sorted
    starts = robot._busy_starts
    t = arrive_ms
    i = bisect.bisect_right(starts, t) - 1
    # inside (possibly chained) busy stretches: wait until they end
    while i >= 0 and i < len(busy) and busy[i][0] <= t < busy[i][1]:
        t = busy[i][1]
        i = bisect.bisect_right(starts, t) - 1
    t += rng.uniform(0, 10)                 # the loop reads WiFi every 5-12 ms
    if rng.random() < slow_prob:
        t += rng.uniform(*slow_ms)          # WiFi stack held it up
    return t


def add_clock_checks(robots, t_first, t_last, cfg, rng):
    """The console's clock loop: returns [(t_send, robot_name, token)] and adds the
    robots' answers (H,CLOCK + D,CLOCK lines) to their sent lines."""
    net = cfg.net
    for r in robots:
        r.busy.sort()
        r._busy_sorted = r.busy
        r._busy_starts = [b[0] for b in r.busy]
    sends = []
    token = 0
    last = {}
    t = t_first
    while t < t_last:
        cursor = t
        for r in robots:
            if t < r.t_hello:
                continue
            if t - last.get(r.name, -1e9) < CLOCK_EVERY_S:
                continue
            last[r.name] = t
            for _ in range(3):
                token += 1
                tok = "c%d" % token
                t_send = cursor + rng.uniform(0.0001, 0.0005)
                sends.append((t_send, r.name, tok))
                d1 = (net.oneway_base_ms + rng.expovariate(1.0 / net.oneway_exp_ms)) / 1000
                arrive_ms = r.ms_at(t_send + d1)
                reply_ms = _poll_delay_ms(r, arrive_ms, rng, net.clock_slow_prob, net.clock_slow_ms)
                if rng.random() >= net.clock_loss:
                    r.lines.append([reply_ms, "H,CLOCK,token,robot_ms,boot"])
                    r.lines.append([reply_ms + 0.2, "D,CLOCK,%s,%d,%04X" % (tok, int(reply_ms), r.boot_id)])
                cursor = t_send + 0.05                # time.sleep(0.05) between questions
        t += 1.0 + rng.uniform(0.0005, 0.003)     # stop.wait(1.0) plus the loop's own time
    return sends


def to_packets(robot, cfg, rng):
    """[(t_arrive, payload, ip)] for one robot, after numbering, delays and losses."""
    net = cfg.net
    robot.lines.sort(key=lambda x: x[0])
    out = []
    seq = 0
    prev_arr = -1e18
    bursts = []
    if net.burst_per_min > 0 and robot.lines:
        t0, t1 = robot.t_at(robot.lines[0][0]), robot.t_at(robot.lines[-1][0])
        t = t0
        while True:
            t += rng.expovariate(net.burst_per_min / 60.0)
            if t > t1:
                break
            bursts.append((t, t + rng.uniform(*net.burst_ms) / 1000))
    lost = dict(raw=0, other=0)
    for ms, text in robot.lines:
        if "{SEQ}" in text:
            text = text.replace("{SEQ}", str(seq))   # net::seq when RUNEND was formatted
        payload = "%s|%d|%s" % (robot.name, seq, text)
        seq += 1
        t_send = robot.t_at(ms)
        is_raw = text.startswith("D,RAW,")
        if is_raw and rng.random() < net.raw_loss:
            lost["raw"] += 1
            continue
        if any(a <= t_send < b for a, b in bursts):
            lost["raw" if is_raw else "other"] += 1
            continue
        d = net.oneway_base_ms + rng.expovariate(1.0 / net.oneway_exp_ms)
        if rng.random() < net.spike_prob:
            d += rng.uniform(*net.spike_ms)
        t_arr = max(t_send + d / 1000, prev_arr + 0.00002)   # first in, first out
        prev_arr = t_arr
        out.append((t_arr, payload, robot.ip))
    robot.packets_sent = seq
    robot.lost = lost
    return out


def replay_console(outdir, packets, clock_sends, log_path=None):
    """Feed every packet to alvik_console.Session at its arrival time."""
    events = [(t, 1, ("pkt", p, ip)) for t, p, ip in packets] + \
             [(t, 0, ("clk", name, tok)) for t, name, tok in clock_sends]
    events.sort(key=lambda e: (e[0], e[1]))
    fake = _FakeTime()
    real_time, real_now = alvik_console.time, alvik_console.now
    printed = io.StringIO()
    session = alvik_console.Session(outdir, show_rows=False, on_new_robot=lambda n: None)
    alvik_console.time = fake
    alvik_console.now = fake.time
    try:
        with contextlib.redirect_stdout(printed):
            for t, _, ev in events:
                fake.now = t
                if ev[0] == "clk":
                    with session.lock:
                        session.clock_sent[ev[2]] = (ev[1], t)
                else:
                    session.handle(ev[1], ev[2])
    finally:
        alvik_console.time, alvik_console.now = real_time, real_now
        session.close()
    if log_path:
        with open(log_path, "w") as f:
            f.write(printed.getvalue())
    return session


def write_session_json(outdir, start_unix, start_iso):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "session.json"), "w") as f:
        json.dump(dict(start=start_iso, start_unix=start_unix, laptop="SIMULATED", python=platform.python_version(),
                       repo_commit="simulated",
                       note="t_recv in every CSV is the laptop clock (Unix seconds) when the line arrived",
                       simulated=True), f, indent=2)


def iso_from_unix(t, tz_text):
    """ISO time string in the session's time zone (taken from the configured start time)."""
    tz = dt.datetime.fromisoformat(tz_text).tzinfo
    return dt.datetime.fromtimestamp(math.floor(t), tz).isoformat(timespec="seconds")
