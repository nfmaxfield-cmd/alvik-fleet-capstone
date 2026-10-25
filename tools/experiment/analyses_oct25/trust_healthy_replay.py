"""DOE rows "evidence before a sensor's k is used" (e02_trust MIN_N) and the logs half of "other trust limits"
(GATE_K, GATE_MAX_CM, DECAY_PER_SESSION, KC_GATE_SIGMA): false alarms and false refusals on the sessions on disk.

Assumption (stated, not shown): no sensor was faulty in these sessions, so every flag is a false alarm and every
refused wheel fit is a false refusal. The faults-caught half needs the simulator swap test (Sim rows).

How: analyses_oct24/night_trust_replay.replay() (the runner's order: camera at finds, wheels and gyro after each U
run against the pooled fit, front readings), with the current e02_fit (stretches cut at 3 cm jumps) and the trust
constants set per level. Sessions: the 2026-10-23 morning and evening (laptop) and the 2026-10-24 stages 1 and 2 (Pi),
each on a fresh trust file, plus stage 1 -> stage 2 carried (where DECAY_PER_SESSION matters).
KC_GATE_SIGMA: the 2026-10-24 stage-2 KC stops (kc_stops.csv: belief, camera, both sds), refused at 2 / 3 / 4 sds.

Usage: python3 trust_healthy_replay.py
"""
import contextlib
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402
sys.argv = [sys.argv[0]]
import night_trust_replay as NT  # noqa: E402
from night_common import TR  # noqa: E402

SESS = ("oct23-morning", "oct23-evening", "stage1", "stage2")


def hours(name):
    tr = C.track(name)
    ts = [q[0] for v in tr.values() for q in v]
    return (max(ts) - min(ts)) / 3600.0 if ts else 0.0


def run(levels):
    old = {k: getattr(TR, k) for k in levels}
    old_min = dict(TR.MIN_N)
    try:
        for k, v in levels.items():
            if k == "MIN_N":
                TR.MIN_N.update(v)
            else:
                setattr(TR, k, v)
        out = {}
        with contextlib.redirect_stdout(io.StringIO()):
            for name in SESS:
                tr, fits, flags = NT.replay(name)
                out[name] = (fits, flags)
            tr, f1, fl1 = NT.replay("stage1")
            tr, f2, fl2 = NT.replay("stage2", trust=tr)
            out["stage1->2"] = (f2, fl2)
        return out
    finally:
        for k, v in old.items():
            if k != "MIN_N":
                setattr(TR, k, v)
        TR.MIN_N.clear()
        TR.MIN_N.update(old_min)


def summary(label, out, hrs):
    nf = sum(sum(1 for f in out[n][1] if f[3] == "degraded") for n in SESS)
    nref = sum(sum(1 for f in out[n][0] if not f["wheel_ok"]) for n in SESS)
    nfit = sum(len(out[n][0]) for n in SESS)
    per = " ".join("%s %d" % (n, sum(1 for f in out[n][1] if f[3] == "degraded")) for n in SESS)
    carried = sum(1 for f in out["stage1->2"][1] if f[3] == "degraded")
    who = sorted({"%s %s" % (f[2], f[1]) for n in SESS for f in out[n][1] if f[3] == "degraded"})
    print("  %-28s flags %2d (%.1f an hour; %s; stage 2 carried %d)  wheel fits refused %d of %d  [%s]" % (
        label, nf, nf / hrs, per, carried, nref, nfit, ", ".join(who)))


def kc_gate_check():
    rows = C.rows("stage2", "kc_stops.csv")
    print("\nKC_GATE_SIGMA on the stage-2 KC stops (n %d): stops whose belief would be refused" % len(rows))
    for k in (2.0, 3.0, 4.0, 5.0):
        n = 0
        for r in rows:
            bx, by = float(r["own_x"]), float(r["own_y"])
            cx, cy = float(r["cam_corr_x"]), float(r["cam_corr_y"])
            rs, cs = float(r["own_sd_cm"]), float(r["cam_sd_cm"])
            d = math.hypot(cx - bx, cy - by)
            vr, vc = rs ** 2, cs ** 2
            w = vr / (vr + vc)
            if d > k * math.sqrt(vr + vc) and w < TR.KC_GATE_WEIGHT:
                n += 1
        print("  %.0f sds: %d refused" % (k, n))
    print("  camera - belief per stop (cm): %s" % ", ".join(r["camera_minus_own_cm"] for r in rows))
    print("  combined sd per stop (cm): %s" % ", ".join("%.1f" % math.sqrt(float(r["own_sd_cm"]) ** 2 +
                                                                        float(r["cam_sd_cm"]) ** 2) for r in rows))


def main():
    hrs = sum(hours(n) for n in SESS)
    print("sessions: %s, %.1f h of track in all; every flag counted as a false alarm (no known fault)" % (
        ", ".join(SESS), hrs))
    print("\nMIN_N (wheels and gyro; camera 4, front 6 as now):")
    for n in (3, 5, 8):
        summary("wheels/gyro %d" % n, run(dict(MIN_N=dict(wheels=n, gyro=n))), hrs)
    print("MIN_N front:")
    for n in (4, 6, 8, 12):
        summary("front %d" % n, run(dict(MIN_N=dict(front=n))), hrs)
    print("MIN_N camera:")
    for n in (3, 4, 6, 8):
        summary("camera %d" % n, run(dict(MIN_N=dict(camera=n))), hrs)
    print("\nGATE_K (refusals are what it changes):")
    for g in (1.2, 1.5, 2.0):
        summary("GATE_K %.1f" % g, run(dict(GATE_K=g)), hrs)
    print("GATE_MAX_CM:")
    for g in (2.0, 2.5, 3.0):
        summary("GATE_MAX_CM %.1f" % g, run(dict(GATE_MAX_CM=g)), hrs)
    print("DECAY_PER_SESSION (only the carried replay can show it):")
    for d in (0.25, 0.5, 0.75, 1.0):
        summary("DECAY %.2f" % d, run(dict(DECAY_PER_SESSION=d)), hrs)
    kc_gate_check()


if __name__ == "__main__":
    main()
