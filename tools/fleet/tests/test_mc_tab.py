"""Mission Control beside other runs (one address, 2026-10-28): a robot run is refused while another program drives the
robots; the plan preview is cut into the routine's parts."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import mission_control as MC  # noqa: E402


def test_robot_run_refused_while_the_doe_loop_drives(tmp_path):
    s = MC.Session(str(tmp_path / "mc"))
    s.proc_scan = lambda: dict(runner=[], doe=[5], mission=[], ladder=[], plan_run=[6], dry=[])
    with pytest.raises(ValueError, match="The DOE loop"):
        s.start_run({"kind": "mission", "conn": "robots", "dry": False})
    s.proc_scan = lambda: dict(runner=[], doe=[5], mission=[], ladder=[], plan_run=[], dry=[5])  # a simulator loop
    assert s.others_driving() == ""
    s.proc_scan = lambda: dict(runner=[7], dry=[])
    assert s.others_driving() == "An E02 session"


def test_preview_parts_of_mondays_routine(tmp_path):
    s = MC.Session(str(tmp_path / "mc"))
    s.set_map({"floor": {"x_min": 0, "y_min": 0, "x_max": 122, "y_max": 61}})
    pv = s.make_mission({"mode": "program"})["preview"]
    for name, segs in pv["segs"].items():
        blocks = [g["block"] for g in segs]
        for b in ("mow", "meet", "circle", "mow on", "chicken", "spread out"):
            assert b in blocks, (name, blocks)
        assert all(a["t1"] <= b["t0"] + 1e-6 for a, b in zip(segs, segs[1:]))           # in time order
        assert segs[0]["pts"][0] == [round(v, 1) for v in pv["starts"][name]]
        ch = segs[blocks.index("chicken")]
        assert len(ch["pts"]) >= 3                                                        # its own visible path
