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


_PLANS = {}


def monday_plan(table):
    """Monday's default for Left and Right on a table, planned through plan_synced (once per table per test run)."""
    import fleetmap as FM
    import mission_run as MR
    import missions as MS
    if table not in _PLANS:
        m = FM.normalize(MR.table_map(table, ["Left", "Right"]))
        mm = MS.build(m, MS.monday_program(("Left", "Right")), MS.settings({}))
        _PLANS[table] = (MS.plan_synced(mm, over={"separation_cm": 8.0}, t_limit_s=60), mm)
    return _PLANS[table]


@pytest.mark.parametrize("table", ["61x122", "70x130", "76x137"])
def test_chicken_is_played_together(table):
    """Both robots wait for each other at their ready spots and set off together (Nick 2026-10-28: Chicken could not be
    seen; each robot used to play its half when it got there, 13-15 s apart)."""
    import engine_checks as EC
    import missions as MS
    plan, mm = monday_plan(table)
    sy = MS._sync_steps(plan)
    ch = [v for k, v in sy.items() if k.startswith("chicken")]
    assert ch and all(abs(v["Left"][0] - v["Right"][0]) <= MS.SYNC_TOL_S for v in ch)
    assert all(abs(v["Left"][0] - v["Right"][0]) <= MS.SYNC_TOL_S for k, v in sy.items() if k.startswith("meet"))
    sw = {n: swerve_s(r["steps"]) for n, r in plan["robots"].items()}
    assert abs(sw["Left"] - sw["Right"]) < 1.0                          # they swerve at the same moment
    assert EC.plan_gap(plan, mm, gate_cm=8.0)["closest_cm"] >= 8.0       # never under 8 cm


def swerve_s(steps):
    """When a robot starts its swerve: the first move after its run at the other robot ("chicken k swerve")."""
    i = max(k for k, st in enumerate(steps) if (st.get("label") or "").endswith(" swerve"))
    return next(st["t_start"] for st in steps[i + 1:] if st["type"] != "wait")


@pytest.mark.parametrize("table", ["61x122", "70x130", "76x137"])
def test_chicken_robots_pass_each_other(table):
    """Nick 2026-10-28 03:52: "they should drive past each other ... they swerve a little and keep going straight".
    Both set off within 0.6 s and swerve together, never under 8 cm (the whole mission), each passes on its own side
    with the bodies about 10 cm apart, and each ends on the other's side (they swap ends); no waits inside Chicken."""
    import engine_checks as EC
    import fleetmap as FM
    import geometry as G
    import numpy as np
    import planner as P
    plan, mm = monday_plan(table)
    body = FM.body_of(FM.normalize(mm))
    cx = (mm["floor"]["x_min"] + mm["floor"]["x_max"]) / 2.0
    ends, lines = {}, {}
    for n, r in plan["robots"].items():
        st = r["steps"]
        ready = [s_ for s_ in st if (s_.get("label") or "").endswith(" ready") and s_["type"] != "wait"][-1]
        last = max(k for k, s_ in enumerate(st) if (s_.get("label") or "").endswith(" pass"))
        ends[n] = (ready["to"][0], st[last]["to"][0])
        lines[n] = st[last]["to"][1]
        assert (ends[n][0] - cx) * (ends[n][1] - cx) < 0, (n, ends[n])        # ends on the other side of the middle
        i0 = max(k for k, s_ in enumerate(st) if (s_.get("label") or "").endswith(" swerve"))
        assert not [s_ for s_ in st[i0 + 1:last + 1] if s_["type"] == "wait"]   # no waits once they swerve
    # passing on opposite sides of the chicken line, the centre lines pass gap + body width apart
    assert abs(lines["Left"] - lines["Right"]) == pytest.approx(10.0 + body.width, abs=0.05)
    assert abs(ends["Left"][1] - ends["Right"][0]) < 1.0 and abs(ends["Right"][1] - ends["Left"][0]) < 1.0
    # the closest two bodies while they pass: the pass gap (10 cm), never under 8
    t0 = min(swerve_s(r["steps"]) for r in plan["robots"].values())
    ts = np.arange(t0, t0 + 12.0, 0.05)
    g = G.body_gaps(body, np.array([P.pose_at(plan, "Left", t) for t in ts]),
                    np.array([P.pose_at(plan, "Right", t) for t in ts]))
    assert 9.0 <= float(g.min()) <= 10.5
    assert EC.plan_gap(plan, mm, gate_cm=8.0)["closest_cm"] >= 8.0


def test_chicken_shape_and_settings():
    """The swerve's geometry (missions.chicken_shape): 70 deg keeps the planner's 9 cm at a 15 cm swerve; a shallow jog
    is made steeper by itself; a pass gap under the floor is said in the notes; the retired aside setting is
    skipped, not refused (an old Mission Control settings.json)."""
    import fleetmap as FM
    import mission_run as MR
    import missions as MS
    m = FM.normalize(MR.table_map("61x122", ["Left", "Right"]))
    sh = MS.chicken_shape(m, MS.settings({}))
    assert sh["jog_deg"] == 70.0 and sh["sidestep_cm"] == pytest.approx(9.75) and sh["swerve_clear_cm"] >= 9.5
    assert not sh["notes"]
    shallow = MS.chicken_shape(m, MS.settings({"chicken_jog_deg": 45}))
    assert shallow["jog_deg"] > 45 and shallow["swerve_clear_cm"] >= 9.5 and shallow["notes"]
    tight = MS.chicken_shape(m, MS.settings({"chicken_pass_gap_cm": 6}))
    assert any("wait for the other" in n for n in tight["notes"])
    assert "chicken_aside_cm" not in MS.settings({"chicken_aside_cm": 20})
    with pytest.raises(ValueError):
        MS.settings({"chicken_nonsense": 1})
