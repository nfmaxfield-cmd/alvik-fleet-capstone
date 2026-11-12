"""The hard simulator (2026-11-12): realism profiles measured from the 11 Nov home logs, the harsh faults, and the
regressions the stress batches found. Fast tests only (a few simulated minutes each); the long batches are
tools/fleet/tests/long_hardsim.py (a separate command)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.join(os.path.dirname(FLEET), "analysis")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fastsim_realism as R      # noqa: E402
import freeroam_fastsim as F     # noqa: E402
import realism_measure as RM     # noqa: E402

HS = R.make_class(F)


def _sim(profile="measured", seed=1, hours=0.05, faults=None, **kw):
    return HS(seed, hours, profile=profile, faults=faults or {}, **kw)


# -- the measures (one definition for real logs and the simulator) ------------------------------------------------
def test_picture_spells_gaps_and_unnamed():
    pics = [i * 0.5 for i in range(40)]
    seen = {"A": [p for p in pics if not 5.0 <= p <= 6.0]}           # missing 3 pictures in a row
    named = {"A": [p for p in pics if p < 10.0 or p >= 14.0]}          # 4 s unnamed while seen
    out = RM.picture_spells(pics, seen, named, ["A"])
    assert len(out["gaps"]) == 1 and abs(out["gaps"][0] - 2.0) < 1e-9
    assert len(out["unnamed"]) == 1 and abs(out["unnamed"][0] - 4.5) < 1e-9
    # a normal picture interval is never an unnamed spell, even at 1.5 pictures a second
    slow = [i / 1.5 for i in range(30)]
    assert RM.picture_spells(slow, {"A": slow}, {"A": slow}, ["A"])["unnamed"] == []


def test_quiet_spells_shared_and_climbs():
    by = {"A": [(t / 10.0, q) for t, q in enumerate([0.1, 0.5, 1.1, 1.4, 0.1, 1.2, 0.2])],
          "B": [(t / 10.0, q) for t, q in enumerate([0.1, 0.2, 1.0, 1.3, 0.1, 0.1, 0.1])]}
    qs, spells, shared = RM.quiet_spells(by)
    assert [round(s[3], 1) for s in spells if s[0] == "A"] == [1.4, 1.2]
    assert shared >= 2


def test_qsample_stays_in_range():
    import random
    q = dict(p5=1.0, p50=2.0, p95=3.0, max=4.0)
    rng = random.Random(1)
    v = [R.qsample(q, rng) for _ in range(2000)]
    assert 1.0 <= min(v) and max(v) <= 4.0
    assert abs(sorted(v)[1000] - 2.0) < 0.1


# -- the profiles -------------------------------------------------------------------------------------------------
def test_profiles_load_and_easy_is_the_old_simulator():
    p = R.load_profile("measured")
    assert p["kind"] == "measured" and p["delay_windows"] and p["pictures_s"]["p50"] < 3
    assert R.load_profile("harsh")["kind"] == "harsh"
    a = F.Sim(3, 0.01, camera="detections").run()
    b = _sim("easy", seed=3, hours=0.01, camera="detections").run()
    for k in ("moves", "watchdog_stops", "min_gap_cm", "contacts"):
        assert a[k] == b[k], k


def test_measured_profile_reproduces_delay_pictures_and_stalls():
    s = _sim(hours=0.05)
    s.run()
    m = s.measure_sim()
    real = R.load_profile("measured")
    assert 1.0 <= m["pictures_s"]["p50"] <= 2.5                          # real 1.6 a second
    assert 0.35 <= m["fix_delay_s"]["p50"] <= 0.8                        # real 0.53 s
    assert m["fix_delay_s"]["p90"] >= 0.8                                # real 1.14 s
    assert m["stalls"]["per_robot_h"] > 100 and m["stalls"]["shared_pct"] > 50   # real 371 / h, 82 % shared
    assert real["stall_pi_per_h"] > 100


def test_stall_holds_fixes_and_robot_messages_then_delivers_them_late():
    s = _sim(hours=0.02)
    s.run()
    late = [d for d in s.m_delay if d > 1.5]
    assert late, "fixes that waited out a stall arrive late"
    a, b = s.stalls[0]
    q = [v for t, v in s.m_quiet["Left"] if a + 0.3 <= t <= b]
    assert q and max(q) >= (b - a) - 0.15                               # nothing reached the robot meanwhile


def test_hung_robot_wheels_turn_body_still():
    s = _sim(hours=0.04, faults={"hung": True})
    n, t0, L = s.hung_at
    s.hours = (t0 + 20.0) / 3600.0
    seen = {}

    def watch():
        b = s.bots[n]
        if t0 + 2 <= s.t <= t0 + 18:
            seen.setdefault("truth", []).append((b.x, b.y))
            seen.setdefault("own", []).append((b.ox, b.oy))
    tr = s.truth

    def truth():
        tr()
        watch()
    s.truth = truth
    s.run()
    tx = seen["truth"]
    assert math.dist(tx[0], tx[-1]) < 0.5
    assert s.m_slip_s.get(n, 0.0) > 15.0


def test_restart_silences_then_turns_its_own_heading():
    s = _sim(hours=0.03, faults={"restart": True})
    s.restart["t"] = 30.0
    s.restart["dur"] = 8.0
    s.run()
    assert s.restart["done"] and "hdg_off" in s.restart
    kinds = [e["kind"] for e in s.kept.get("events", [])]
    assert "restarting (sim)" in kinds and "restarted (sim)" in kinds


def test_harsh_profile_turns_every_fault_on():
    s = _sim("harsh", hours=0.01)
    assert set(s.faults) == set(R.HARSH_FAULTS)
    assert s.cam_clock == 0.4 and s.dark and s.wrong and s.missing and s.hung_at and s.restart
    assert math.hypot(*s.cam_fault["shift"]) == 5.0


# -- regressions the stress batches found (12 Nov). xfail = still open on the code tested; they pass once fixed ------
import pytest  # noqa: E402


def _follow_restart(seed, t_restart=60.0, dur=10.0, minutes=4.0):
    s = _sim(seed=seed, hours=minutes / 60.0, faults={"restart": True})
    s.restart.update(t=t_restart, dur=dur)
    rec = []
    tr = s.truth

    def truth():
        tr()
        if s.restart["done"] and int(s.t * 2) != int((s.t - F.DT) * 2):
            b = s.bots[s.restart["name"]]
            rec.append((s.t, abs(F.wrap180(b.oth - b.th)), math.hypot(b.ox - b.x, b.oy - b.y)))
    s.truth = truth
    s.run()
    return s, rec


def test_restart_heading_is_checked_before_it_wanders():
    """(e19: fixed, was xfail) hard simulator 12 Nov (seed 2): a robot that restarts (its gyro's heading starts anywhere)
    drove on for about a minute with its own heading 40-70 deg off and its own pose up to 55 cm from the truth. Now its
    pose is sent after the boot and a heading check is asked (freeroam_core.on_restart / boot_watch)."""
    s, rec = _follow_restart(2)
    assert rec and max(d for _, _, d in rec) < 20.0


@pytest.mark.xfail(reason="(e19, partly fixed) hard simulator seed 8: with a wrong camera name for 5 minutes (its "
                          "sightings mirrored) e18 never concluded the camera was wrong (3 contacts). e19's whole-camera "
                          "watch concludes it, but on this seed only after 72 s (robots rarely drove two at once), and "
                          "one robot fell before that. Seeds 1-3: concluded within 5-6 s, no contact, no fall.",
                   strict=False)
def test_wrong_camera_name_mid_run_no_contact():
    s = _sim(seed=8, hours=6 / 60.0, faults={"wrong_names": True})
    s.wrong = (30.0, 330.0)
    s.run()
    kinds = {e["kind"] for e in s.kept.get("events", [])}
    assert "camera disagrees" in kinds or "camera" in kinds  # each robot notices, or the whole camera is not trusted
    assert s.stats["contacts"] == 0 and s.stats.get("falls", 0) == 0
