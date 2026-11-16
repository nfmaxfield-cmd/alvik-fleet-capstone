"""(2026-11-16, Nick) Mission Control's Visit points: a list per robot. The page sends 'Alpha:P01,P05;Bravo:P11' (the
form mission_run already took, e11); the one shared list stays the default and is unchanged. A named robot the wiggle
did not find this run no longer stops the run: the robots found run their own lists."""
import os
import re
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
sys.path[:0] = [FLEET, TOOLS, os.path.join(TOOLS, "sim")]

import mission_run as MR  # noqa: E402


def test_shared_list_unchanged():
    assert MR.parse_visit_order("P01,P02,P03", ["Alpha", "Bravo"]) == {"Alpha": ["P01", "P02"], "Bravo": ["P03"]}
    assert MR.parse_visit_order("P01,p5, 11,P05", ["Alpha"]) == {"Alpha": ["P01", "P05", "P11", "P05"]}


def test_a_list_per_robot_with_repeats_and_any_case():
    got = MR.parse_visit_order("alpha:P01,P05,P01;BRAVO:P20,P15", ["Alpha", "Bravo"])
    assert got == {"Alpha": ["P01", "P05", "P01"], "Bravo": ["P20", "P15"]}


def test_a_named_robot_not_found_leaves_the_others_running(capsys):
    got = MR.parse_visit_order("Alpha:P01,P05;Bravo:P11,P12", ["Alpha"])
    assert got == {"Alpha": ["P01", "P05"]}
    assert "Bravo was not found in this run" in capsys.readouterr().out


def test_every_named_robot_missing_is_still_refused():
    import pytest
    with pytest.raises(ValueError):
        MR.parse_visit_order("Zulu:P01;Yankee:P02", ["Alpha"])


def test_per_robot_lists_reach_visit_orders(tmp_path):
    import sites as SI
    a = types.SimpleNamespace(site_=SI.load("school-conference"), visit_order="Alpha:P01,P05;Bravo:P11,P12",
                              visit_keep_unreadable=True, readmap_=None)
    a.readmap_ = types.SimpleNamespace(readable=lambda x, y: (True, "test"))
    assert MR.visit_orders(a, ["Alpha", "Bravo"], str(tmp_path)) == {"Alpha": ["P01", "P05"], "Bravo": ["P11", "P12"]}


def test_page_sends_the_per_robot_form_and_keeps_the_shared_list():
    s = open(os.path.join(FLEET, "mission_control_table.html")).read()
    assert 'n + ":" + lists[n].join(",")).join(";")' in s          # Alpha:P01,P05;Bravo:P11
    assert 'body.order = order.join(",")' in s                       # the shared list, as before
    assert re.search(r"let vpEach = false", s)                       # shared is the default
