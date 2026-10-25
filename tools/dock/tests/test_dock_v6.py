"""Dock v6's numbers (no CAD build: the plain arithmetic in build_dock_v6.py): the pads on the battery cover, two
springs per plate, the wider mouth (Nick, 2026-10-25 13:03)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "hardware", "dock"))

import build_dock_v6 as V       # noqa: E402
import build_dock_v5 as V5      # noqa: E402


def test_the_carrier_sits_on_the_cover_clear_of_the_screw_and_the_caster():
    assert V.CARRIER_X1_MM <= V.COVER_FRONT_X_MM - 0.5 and V.CARRIER_X0_MM >= V.COVER_BACK_X_MM + 0.5
    assert V.CARRIER_HALF_W_MM <= V.COVER_HALF_W_MM - 0.5
    assert (V.COVER_SCREW_X_MM - V.SCREW_HEAD_R_MM) - V.CARRIER_X1_MM >= 2.0          # the screw stays free
    assert V.PAD_Y_MM - V.SLOT_FLARE_HALF_W_MM - V.NOTCH_HALF_W_MM >= 0.8             # lanes clear the caster notch
    assert V.COVER_BACK_X_MM < V.PAD_X0_MM < V.PAD_X_MM < V.PAD_X1_MM < V.COVER_FRONT_X_MM
    # the crown lands on the pad's middle with room for the photo's error
    assert V.PAD_X1_MM - V.PAD_X_MM - 1.0 >= 1.5


def test_two_springs_stiffer_and_the_lift_stays_in_budget():
    assert V.SPRING_N == 2 and V.roll_k() > 0
    assert V.contact_k() > V5.contact_k()                       # stiffer than v5's single spring
    F = V.contact_k() * V.PRESS_MM
    assert 0.2 <= F <= 0.35
    assert V.lift_frac(2 * F, V.PAD_X_MM) < 0.45
    cur = V.lift_curve_v6([i * 0.5 for i in range(121)])
    assert max(r[1] for r in cur) < 0.55
    assert all(r[1] == 0.0 for r in cur if r[0] > 12)          # nothing touches until the last 12 mm
    low = V.lift_curve_v6([i * 0.5 for i in range(121)], dz=-0.4)
    assert max(r[1] for r in low) < 0.75


def test_springs_fit_side_by_side_and_never_go_solid():
    assert V.SPRING_Y_OFF_MM - V.SPRING_POCKET_R_MM >= 0.5                 # a printed wall between the pockets
    assert V.PRONG_HALF_W_MM - (V.SPRING_Y_OFF_MM + V.SPRING_POCKET_R_MM) >= 1.2
    assert V.SPRING_FREE_MM - V.DELTA_MAX_MM * V.A_RATIO >= V.SPRING_SOLID_MM
    assert V.Z_UNDER_MM - V.CROWN_FREE_Z_MM >= 0.8


def test_the_mouth_catches_more_and_the_arms_still_hold_the_crowns():
    play = V.ARM_IN_HALF_W_MM - V.D.BRACKET_W_MM / 2
    assert V.FLARE_MM + play >= 12.0 > V5.FLARE_MM + play
    x_bar_at_crown = V.X_CROWN_MM - (V.CARRIER_X0_MM - V.BAR_FACE_X_MM)
    assert V.inner_half_w(x_bar_at_crown) == V.ARM_IN_HALF_W_MM
    assert V.horn_out_mm() <= V.V2.POST_STANDOFF_MM - 3.0


def test_photo_mapping_puts_the_caster_where_the_photo_shows_it():
    u, v = V.photo_px(V.CASTER_X_MM, 0.0)
    assert abs(u - 605) < 15 and abs(v - 1295) < 20             # the ball's centre on Nick's photo
