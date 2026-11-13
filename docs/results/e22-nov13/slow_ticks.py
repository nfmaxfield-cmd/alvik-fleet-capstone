#!/usr/bin/env python3
# e22 (Claude's tracing for the report; simulator and logs only): Which controller methods the slowest 1 in 100 ticks spend their time in (inclusive, slow ticks only).
# Run from the repo root (the report's Appendix B has the commands). Changelog: e22-2026-11-13a first version.
"""Which controller methods the slowest 1 % of ticks spend their time in (inclusive time per method, slow ticks only)."""
import sys, time, json, collections, functools
sys.path.insert(0, sys.argv[1])
import freeroam_fastsim as F, fastsim_realism as R, freeroam_core as FC
cur = collections.Counter()
stack = []
NAMES = [n for n, v in vars(FC.Controller).items() if callable(v) and not isinstance(v, (staticmethod, classmethod)) and not n.startswith("__")]
def wrap(name, fn):
    @functools.wraps(fn)
    def w(*a, **k):
        if name in stack:
            return fn(*a, **k)
        stack.append(name)
        t0 = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            cur[name] += time.perf_counter() - t0
            stack.pop()
    return w
for n in NAMES:
    if n != "tick":
        setattr(FC.Controller, n, wrap(n, getattr(FC.Controller, n)))
ticks = []
orig = FC.Controller.tick
def tick(self, t):
    cur.clear()
    a = time.perf_counter(); orig(self, t); d = time.perf_counter() - a
    ticks.append((d, dict(cur)))
FC.Controller.tick = tick
prof = R.load_profile(sys.argv[2]); Cls = R.make_class(F)
s = Cls(int(sys.argv[3]), float(sys.argv[4]), profile=sys.argv[2], prof=prof, camera="detections", sensors=True,
        edge_physics=True)
import os
if os.environ.get("ASYNC"):
    for t_ in s.ctrl.trust.values(): t_.async_fit = True
s.run()
ticks.sort(key=lambda q: q[0])
n = len(ticks)
slow = ticks[int(0.99 * n):]
print("ticks", n, "p50 %.2f p95 %.2f p99 %.2f max %.1f ms" % tuple(1000 * ticks[int(p * (n - 1))][0] for p in (.5, .95, .99, 1.0)))
agg = collections.Counter()
for d, c in slow:
    for k, v in c.items():
        agg[k] += v
tot = sum(d for d, _ in slow)
for k, v in agg.most_common(25):
    print("%-28s %5.1f %% of slow-tick time" % (k, 100 * v / tot))
