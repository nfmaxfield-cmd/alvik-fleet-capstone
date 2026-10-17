#!/usr/bin/env python3
"""WiFi messaging test (experiment E3): how fast and how reliably do messages
get to each robot and back?

    python tools/commtest.py                       # every robot heard, 5 msg/s, 300 messages each
    python tools/commtest.py --rate 20 --count 1000
    python tools/commtest.py --robots Left --rate 5
    python tools/commtest.py --robots Left,Center,Right --rate 20 --count 1000 --label E3-3robots-20hz

Each message is a small "clock" question the robot answers at once. For every
message it records when it left the laptop and when the answer came back:

  round trip (ms)    time there and back; one-way delay is about half of it
  jitter (ms)        how much the round trip varies (standard deviation)
  delivery (%)       answers received / questions sent, both directions combined
  robot -> laptop    separately, packets lost on the way back, from the robots'
                     own packet counters

Robots answer between other work (about every 10 ms when idle), so the round
trip includes that wait: this is the delay a real command would see.

Writes logs/<date-time>/commtest.csv (one row per message) and prints a table.
With several robots, all are tested at the same time (as E3's "three robots"
condition needs).
"""
import argparse
import csv
import datetime as dt
import math
import os
import statistics as st
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alvik_console import Link, now  # noqa: E402


def pct(vals, p):
    if not vals:
        return float("nan")
    v = sorted(vals)
    k = (len(v) - 1) * p / 100
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


class Answers:
    def __init__(self):
        self.lock = threading.Lock()
        self.got = {}   # token -> (t_recv, robot_ms)

    def __call__(self, name, kind, rtype, vals):
        if kind == "D" and rtype == "CLOCK" and vals and vals[0].startswith("t"):
            with self.lock:
                self.got.setdefault(vals[0], (now(), vals[1] if len(vals) > 1 else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", help="comma-separated names (default: every robot heard)")
    ap.add_argument("--rate", type=float, default=5, help="messages per second, per robot")
    ap.add_argument("--count", type=int, default=300, help="messages per robot")
    ap.add_argument("--label", default="", help="added to the output file name, e.g. E3-1robot-5hz")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    ap.add_argument("--outdir", help="log folder (default logs/<date-time>)")
    ap.add_argument("--wait", type=float, default=2.0, help="seconds to wait for late answers at the end")
    args = ap.parse_args()

    outdir = args.outdir or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    link = Link(outdir, args.robot)
    link.session.quiet = True
    ans = Answers()
    link.session.listeners.append(ans)

    want = [r.strip() for r in args.robots.split(",")] if args.robots else None
    t0 = time.time()
    while time.time() - t0 < 10:
        have = link.robots()
        if have and (want is None or set(want) <= set(have)):
            break
        time.sleep(0.5)
    robots = want or link.robots()
    missing = [r for r in robots if r not in link.robots()]
    if not robots or missing:
        print(f"Not connected: {', '.join(missing) or 'no robots heard'}. Heard: {', '.join(link.robots()) or 'none'}")
        link.close()
        return 1
    time.sleep(1.0)
    print(f"Testing {', '.join(robots)}: {args.count} messages each at {args.rate:g}/s "
          f"(about {args.count / args.rate:.0f} s)")

    sent = {}            # token -> (robot, t_send)
    lock = threading.Lock()
    with link.session.lock:
        before = {n: (r["received"], r["dropped"]) for n, r in link.session.robots.items()}

    def sender(robot, offset):
        period = 1.0 / args.rate
        time.sleep(offset)       # spread robots' messages so they don't all leave at once
        nxt = time.time()
        for k in range(args.count):
            tok = f"t{robot}{k}"
            with lock:
                sent[tok] = (robot, now())
            link.send_name(robot, f"clock {tok}")
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))

    threads = [threading.Thread(target=sender, args=(r, i / (args.rate * len(robots))))
               for i, r in enumerate(robots)]
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.5)
            with lock, ans.lock:
                n, a = len(sent), sum(1 for k in sent if k in ans.got)
            print(f"\r  sent {n}, answered {a}", end="", flush=True)
        time.sleep(args.wait)
    except KeyboardInterrupt:
        print("\n  stopped early")
    print()

    with link.session.lock:
        after = {n: (r["received"], r["dropped"]) for n, r in link.session.robots.items()}
    os.makedirs(outdir, exist_ok=True)
    name = f"commtest_{args.label}.csv" if args.label else "commtest.csv"
    path = os.path.join(outdir, name)
    rows = []
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "token", "t_send", "t_recv", "rtt_ms", "answered", "rate_hz", "robots_tested"])
        with lock, ans.lock:
            for tok, (robot, ts) in sorted(sent.items(), key=lambda kv: kv[1][1]):
                got = ans.got.get(tok)
                rtt = (got[0] - ts) * 1000 if got else None
                rows.append((robot, rtt))
                w.writerow([robot, tok, f"{ts:.4f}", f"{got[0]:.4f}" if got else "",
                            f"{rtt:.2f}" if rtt is not None else "", 1 if got else 0, args.rate, len(robots)])

    print(f"\n{'robot':10s} {'sent':>6s} {'delivery':>9s} {'rtt p50':>8s} {'p95':>7s} {'p99':>7s} {'max':>7s} "
          f"{'jitter':>7s} {'robot->laptop loss':>19s}")
    for robot in robots:
        r = [x for rb, x in rows if rb == robot]
        ok = [x for x in r if x is not None]
        b, a = before.get(robot, (0, 0)), after.get(robot, (0, 0))
        got, dropped = a[0] - b[0], a[1] - b[1]
        back_loss = 100 * dropped / (got + dropped) if got + dropped else float("nan")
        jit = st.stdev(ok) if len(ok) > 1 else float("nan")
        print(f"{robot:10s} {len(r):6d} {100 * len(ok) / len(r) if r else 0:8.1f}% {pct(ok, 50):8.1f} "
              f"{pct(ok, 95):7.1f} {pct(ok, 99):7.1f} {max(ok) if ok else float('nan'):7.1f} {jit:7.1f} "
              f"{back_loss:18.1f}%")
    print("\nTimes in ms. Delivery counts a message as delivered only if its answer came back.")
    print(f"Saved {path}")
    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
