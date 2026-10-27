"""--no-boards (Nick, 2026-10-26 22:49: both boards off for edge data; Monday's school table has none): all four sides
are open edges from the start, nothing reads a missing board, and the plan's switch implies the edge finds."""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "recognition"))

import e02 as E  # noqa: E402
import e02_plan as PL  # noqa: E402


def _no_boards():
    b = PL.Board(xboard_side=1, yboard_side=1)
    b.missing = {"x", "y"}
    return b


def test_four_open_edges():
    edges = sorted(_no_boards().open_edges())
    b = PL.Board(xboard_side=1, yboard_side=1)
    assert len(edges) == 4
    assert sorted(set(edges)) == sorted([("x", b.x0), ("x", b.x1), ("y", b.y0), ("y", b.y1)])


def test_front_check_sees_no_missing_board():
    """2026-10-26 dry run 1: the front sensor was compared with boards that were not there (false lost holds, and
    the front sensor marked bad)."""
    b = _no_boards()
    r = types.SimpleNamespace(board=b, st={"positions": {}})
    # facing the high-x side from the middle: with the boards there, the y-board is ahead; without, nothing is
    assert E.Runner.ahead_cm(r, "Left", (61.0, 61.0), 0.0) is None
    b.missing = set()
    ahead = E.Runner.ahead_cm(r, "Left", (61.0, 61.0), 0.0)
    assert ahead is not None and ahead[1] == "the y-board"


def test_board_recheck_skips_missing_boards():
    """2026-10-26 dry run 2 ended on a KeyError: no board lines were measured, because there are no boards."""
    r = types.SimpleNamespace(board=_no_boards(), feed=types.SimpleNamespace(last_img=object()), st={"setup": {}})
    E.Runner.boards_still(r)          # nothing to re-measure, and no error


def test_no_boards_round_trips():
    b = _no_boards()
    b2 = PL.Board.from_dict(b.to_dict())
    assert b2.missing == {"x", "y"} and len(b2.open_edges()) == 4


def test_a_healthy_turn_swing_away_from_the_edge_does_not_hold():
    """2026-10-27 stage 1: Left's half turn 13 cm from an edge swung its camera centre 5.5 cm and set it aside."""
    assert E.spin_allowance_cm(2) < 3.0 and 8.0 < E.spin_allowance_cm(180) < 9.0
    assert E.spin_allowance_cm(-90) == E.spin_allowance_cm(90)
