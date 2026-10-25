"""The sensor-trust fix (2026-10-25) on the REAL robots' sessions, logs only: false flags, and the real run-to-run wheel
scale spread (what e02_trust RUN_SPREAD_PRIOR stands in for).

Assumption (stated, as in trust_healthy_replay.py): no sensor was faulty in these sessions, so every flag is false.

How: analyses_oct24/night_trust_replay's events in the runner's order (camera at finds, a U run's fit, front
readings), plus each run's wheel scale (e02.run_scales), through the current e02_trust. Each session cold (fresh
trust), then chained (each session the next one's history: morning -> evening -> stage 1 -> stage 2).

    python3 tools/experiment/analyses_oct25/trust_fix_real_replay.py
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "analyses_oct24"))
sys.path.insert(0, os.path.dirname(HERE))
sys.argv = [sys.argv[0]]
with contextlib.redirect_stdout(io.StringIO()):
    import night_trust_replay as NT  # noqa: E402
from night_common import TR, FIT  # noqa: E402
import night_common as N  # noqa: E402
import e02 as E  # noqa: E402

SESS = ("oct23-morning", "oct23-evening", "stage1", "stage2")


def replay(name, trust=None):
    st = N.state(name)
    trust = trust or TR.SensorTrust()
    trust.new_session()
    ev = sorted(NT.find_events(name, False) + NT.fit_events(name) + NT.front_events(name), key=lambda e: e[0])
    data, prev, flags, runs = {}, {}, [], {}
    robots = list(st["robots"])
    for t, kind, robot, v in ev:
        if kind == "camera":
            trust.camera_at_find(robot, v[0], v[1], t)
        elif kind == "front":
            trust.front_vs_map(robot, v, t)
        else:
            order, obs = v
            pr, measured = NT.prior(st, robot)
            d = data.setdefault(robot, [])
            d += obs
            res = FIT.fit_robot(d, prev.get(robot), prior=pr, prior_sd=dict(gyro_drift_dps=0.1) if measured else None,
                                measured_drift=pr["gyro_drift_dps"] if measured else None,
                                max_rms_cm=trust.wheel_gate_cm())
            if obs:
                rc, rd = NT.stretch_rms_against(obs, res.get("fitted") or {}, pr)
                trust.stretch_fit(robot, rc, rd, len(obs), t)
                for _r, sc, ln in E.run_scales(obs):
                    if not hasattr(trust, "run_scale") or trust.run_scale(robot, sc, ln, t):
                        runs.setdefault(robot, []).append(sc)
            if res["values"] and res["accepted"]:
                prev[robot] = res["values"]
        for s_, r_, what, kk in trust.update_flags(robots):
            flags.append((s_, r_, what, round(kk, 2), trust.reason(s_, r_) if what == "degraded" and hasattr(
                trust, "reason") else ""))
    return trust, flags, runs, robots


def main():
    print("| Session | Flags, cold | Flags, chained (earlier sessions as history) | Run-to-run wheel spread per robot |")
    print("|---|---|---|---|")
    chain = None
    lines = []
    for name in SESS:
        t1, f1, runs, robots = replay(name)
        chain, f2, _, _ = replay(name, chain)
        rs = getattr(TR, "rel_spread", None)
        sp = ", ".join("%s %.1f%% (%d)" % (r, 100 * rs(v), len(v)) for r, v in sorted(runs.items())
                       if len(v) >= 2 and rs) if rs else ""
        fw = lambda f: "; ".join("%s/%s %.1f" % (s, r, k) for s, r, w, k, _ in f if w == "degraded") or "none"
        print("| %s | %s | %s | %s |" % (name, fw(f1), fw(f2), sp or "-"))
        lines.append((name, f1, f2, t1.summary(robots), chain.summary(robots), robots))
    print()
    for name, f1, f2, t1, t2, robots in lines:
        print("%s:" % name)
        for lab, f, t in (("cold", f1, t1), ("chained", f2, t2)):
            for s, r, w, k, why in f:
                print("    %s: %s/%s %s %.1f %s" % (lab, s, r, w, k, why))
            for ln in t:
                print("    %s   %s" % (lab, ln))


if __name__ == "__main__":
    main()
