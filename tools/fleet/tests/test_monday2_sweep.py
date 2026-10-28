"""Monday 2-robot sweep (2026-10-28): its conditions, mission_run's new pass-through options, sim_shim's live-camera
stall / side-only and restart hooks."""
import json
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(FLEET, "sweeps"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import monday2_sweep as M2   # noqa: E402


def test_conditions_are_repeatable_and_gyro_runs_paired():
    a = M2.cond_args("robot dies", "61x122", 2)
    assert a == M2.cond_args("robot dies", "61x122", 2)
    u = M2.cond_args("gyro 7.5% off, uncorrected", "76x137", 1)
    c = M2.cond_args("gyro 7.5% off, corrected", "76x137", 1)
    assert u[2] == c[2]                                   # the same robots' errors, with and without the correction
    assert not any("gyro_scale" in x for x in u[1])
    assert "--rp-arg=set gyro_scale 1.0750" in c[1]
    w = M2.cond_args("wheels 3-5% off", "61x122", 1)[0]["_wheel"]
    assert all(3.0 <= abs(v) <= 5.0 for v in w.values())
    r = M2.cond_args("restart mid-run", "61x122", 3)[0]["restart"]
    assert M2.OUT_T[0] <= r["t"] <= M2.OUT_T[1] and M2.BOOT_S[0] <= r["boot_s"] <= M2.BOOT_S[1]


def test_job_order_ends_first_then_doe_then_middle_sizes():
    a = types.SimpleNamespace(seeds="1", sizes=",".join(M2.SIZES), levels="8,5", only=[])
    jobs = M2.jobs_for(a)
    first_mid = next(i for i, j in enumerate(jobs) if j[1] in ("66x127", "70x130"))
    first_doe = next(i for i, j in enumerate(jobs) if j[3] == 5.0)
    assert first_doe < first_mid
    assert all(j[1] in ("61x122", "76x137") for j in jobs[:first_doe])
    assert {j[0] for j in jobs if j[1] == "66x127"} == M2.FULL_SIZES


def test_mission_run_settings_lower_the_safety_net(tmp_path):
    import mission_run as MR
    d = str(tmp_path / "m")
    MR.main(["--table", "61x122", "--plan-only", "--session", d, "--min-gap-cm", "6.5",
             "--setting", "separation_cm=6.5", "--setting", "chicken_min_cm=6.5"])
    prog = json.load(open(os.path.join(d, "program.json")))
    assert [b["min_cm"] for b in prog["blocks"] if b["block"] == "chicken"] == [6.5]
    mm = json.load(open(os.path.join(d, "mission.json")))
    assert mm["mission"]["planner"]["separation_cm"] == 6.5
    plan = json.load(open(os.path.join(d, "plan.json")))
    assert 6.5 <= plan["summary"]["min_separation_cm"] < 12.0      # 8 cm plans 12.0 here: the lower net is used


def test_live_stall_stale_and_side_only_hooks():
    import table_cam as TC
    import sim_shim as SS
    o_t, o_s = TC.TableCamera.render, TC.SideCamera.render
    try:
        sim = types.SimpleNamespace(args=types.SimpleNamespace(fix_sigma=0.3), m={"floor": {}},
                                    cam_gap=lambda n, t: False, send_json=lambda m: None, robots={}, t_plan0=None,
                                    drops=[], event=lambda e: None)
        SS.install(sim, {"live_stall_s": 2.0, "live_stall_mode": "stale", "live_stall_first_s": 1.0,
                         "side_only": True}, 1)
        cam = TC.TableCamera((0, 0, 122, 61), px_per_cm=2.34)
        robot = [(30.0, 30.0, 0.0, (0, 0, 0), (0, 0, 0))]
        empty = o_t(cam, ())
        a = cam.render(robot)
        assert abs(a.astype(int) - empty.astype(int)).max() <= 20      # side only: the overhead sees no robot (noise)
        sim.t_plan0 = time.time() - 1.5                                   # inside the stall: the same picture again
        assert cam.render(robot) is a
        sim.t_plan0 = time.time() - 3.5                                   # after it: a new picture
        assert cam.render(robot) is not a
    finally:
        TC.TableCamera.render, TC.SideCamera.render = o_t, o_s


def test_dance_started_report_lost_is_taken_from_follow():
    """15 % packet loss (sweep 2026-10-28): the dance's 'started' report was lost and run_plan waited until the time
    limit. A FOLLOW record naming it as the current step now starts it (and a dance until switched off is done)."""
    import run_plan as RP
    c = object.__new__(RP.Coordinator)
    c.log = lambda *a, **k: None
    c.say = lambda s: None
    c.plan_start = 0.0
    steps = [{"type": "turn"}, {"type": "dance"}]
    c.steps = lambda name: steps
    b = RP.Bot("Right")
    b.id_index = {75: 0, 76: 1}
    b.stat = [{"status": "done"}, {"status": "acked"}]
    b.note_clock = lambda t, ms: None
    # FOLLOW: run, ms, step_id (cur), state, queued, last_done, last_failed, x, y, th, sd, th_sd, used, rejected, front
    v = ["run", "1000", "76", "nopose", "0", "75", "0", "40.7", "30.7", "-113.7", "0.3", "1", "0", "0", "150"]
    c.on_follow(b, 5.0, v)
    assert b.stat[1]["status"] == "done" and getattr(b, "dancing", False)
