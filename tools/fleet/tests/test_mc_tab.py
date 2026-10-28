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


@pytest.mark.parametrize("table", ["61x122", "70x130", "76x137"])
def test_chicken_is_played_together(table):
    """Both robots wait for each other at their ready spots and set off together (Nick 2026-10-28: Chicken could not be
    seen; each robot used to play its half when it got there, 13-15 s apart)."""
    import engine_checks as EC
    import fleetmap as FM
    import mission_run as MR
    import missions as MS
    m = FM.normalize(MR.table_map(table, ["Left", "Right"]))
    mm = MS.build(m, MS.monday_program(("Left", "Right")), MS.settings({}))
    plan = MS.plan_synced(mm, over={"separation_cm": 8.0}, t_limit_s=60)
    sy = MS._sync_steps(plan)
    ch = [v for k, v in sy.items() if k.startswith("chicken")]
    assert ch and all(abs(v["Left"][0] - v["Right"][0]) <= MS.SYNC_TOL_S for v in ch)
    assert all(abs(v["Left"][0] - v["Right"][0]) <= MS.SYNC_TOL_S for k, v in sy.items() if k.startswith("meet"))
    sw = {n: next(st["t_start"] for st in r["steps"] if (st.get("label") or "").endswith("swerve"))
          for n, r in plan["robots"].items()}
    assert abs(sw["Left"] - sw["Right"]) < 1.0                          # they swerve at the same moment
    assert EC.plan_gap(plan, mm, gate_cm=8.0)["closest_cm"] >= 8.0       # never under 8 cm
