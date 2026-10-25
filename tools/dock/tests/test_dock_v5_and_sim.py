"""Dock v5's contact numbers (no CAD build: the plain arithmetic in build_dock_v5.py) and the policy simulator's rest
policy."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCK = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(DOCK))
sys.path.insert(0, DOCK)
sys.path.insert(0, os.path.join(REPO, "hardware", "dock"))

import build_dock_v5 as V       # noqa: E402
import charge_policy_sim as S   # noqa: E402


def test_the_contacts_take_well_under_v3s_59_percent_off_the_caster():
    F = V.contact_k() * (V.CROWN_FREE_Z_MM - V.Z_PAD_MM)
    assert V.lift_frac(2 * F, V.PAD_X_MM) < 0.2
    cur = V.lift_curve_v5([i * 0.5 for i in range(121)])
    assert max(r[1] for r in cur) < 0.3
    assert all(r[1] == 0.0 for r in cur if r[0] > 16)          # nothing touches until the last 16 mm
    v3 = V.lift_curve_v3([i * 0.5 for i in range(121)])
    assert max(r[1] for r in v3) > 0.55


def test_the_crown_clears_the_robot_and_the_spring_never_goes_solid():
    assert V.Z_UNDER_MM - V.CROWN_FREE_Z_MM >= 0.8
    assert V.SPRING_FREE_MM - V.DELTA_MAX_MM * V.A_RATIO >= V.SPRING_SOLID_MM


def test_known_lengths_include_the_marker():
    names = {n: v for n, v, _ in V.known_lengths()}
    assert names["marker square pitch"] == 15.0 and any(abs(v - 90.0) < 1e-9 for v in names.values())


def test_rest_policy_keeps_at_most_k_working_and_slow_charging_needs_it():
    old = S.DAYS
    S.DAYS = 2.0
    try:
        p = dict(S.P, cc_rate=40.0)
        r = S.simulate(3, 30, 70, p, "typical", seed=1, rest_k=1)
        assert r["uptime"] <= 1 / 3 + 0.01 and r["min"] > 0
        p = dict(S.P, cc_rate=30.0)          # 0.75 A: too slow for 3 robots working, enough for 1 working + 2 resting
        plain = S.simulate(3, 40, 80, p, "worst")
        rest = S.simulate(3, 30, 70, p, "worst", rest_k=1)
        assert plain["min"] == 0.0 and rest["min"] > S.P["floor"]
        p = dict(S.P, cc_rate=20.0)          # below the energy limit even for one: resting asleep still drains
        assert S.simulate(3, 30, 70, p, "worst", rest_k=1)["min"] == 0.0
    finally:
        S.DAYS = old
