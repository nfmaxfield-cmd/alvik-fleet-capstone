"""Experiment routes, sync wiggle, run exports and the raw bundle, duplicated points (school build, 2026-11-03).

Jay-Sun's training runs: four route orders plus the shakedown, halves swapped every other run, legs over x = 78 in,
one speed; the sync wiggle at the start; the per-frame camera track; the wheel+gyro odometry with no camera correction;
the raw-files zip (nothing from the RFID side); a point is done once under keep going.
"""
import csv
import json
import math
import os
import sys
import types
import zipfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
for p in (FLEET, os.path.dirname(FLEET), os.path.join(os.path.dirname(FLEET), "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)

import experiments as EX     # noqa: E402
import missions as MS        # noqa: E402
import production as PR      # noqa: E402
import run_exports as RX     # noqa: E402
import sites as SI           # noqa: E402
import sticker_points as SP  # noqa: E402

MID_CM = 78 * 2.54


@pytest.fixture(scope="module")
def school():
    return SI.load("school-conference")


def _orders(site, exp, swap=False):
    m, _w, prog, orders = PR.site_production(site, experiment=exp, swap=swap)
    return m, prog, orders


@pytest.mark.parametrize("exp", ["shakedown", "e1", "e2", "e3", "e4", "default"])
@pytest.mark.parametrize("swap", [False, True])
def test_every_point_once_per_run(school, exp, swap):
    m, prog, orders = _orders(school, exp, swap)
    allp = [l for v in orders.values() for l in v]
    assert sorted(allp) == sorted(m["spots"]) and len(allp) == 40 == len(set(allp))
    meta = prog["production"]["experiment"]
    assert meta["id"] == exp and meta["swap_halves"] == swap and meta["orders"] == orders
    assert meta["speed"] == {"cruise_cms": 10.0, "turn_dps": 60.0, "confirm_cms": 5.0}


def test_in_order_and_reversed_follow_the_numbers(school):
    _m, _p, o1 = _orders(school, "e1")
    assert o1["Alpha"] == ["P%02d" % k for k in range(1, 21)] and o1["Bravo"] == ["P%02d" % k for k in range(21, 41)]
    _m, p2, o2 = _orders(school, "e2")
    own = [l for l in o2["Alpha"] if l not in p2["production"]["experiment"]["traded"]["Alpha"]]
    assert own == sorted(own, key=EX.num, reverse=True)


def test_cross_middle_legs_in_every_non_in_order_experiment(school):
    for exp in ("e2", "e3", "e4"):
        for swap in (False, True):
            m, prog, orders = _orders(school, exp, swap)
            homes = {r["name"]: r["start"] for r in m["robots"]}
            for n, labs in orders.items():
                c = EX.crossings(labs, m["spots"], homes[n], MID_CM)
                assert c >= EX.MIN_CROSS_LEGS, (exp, swap, n, c)
                assert prog["production"]["experiment"]["crossings"][n] == c
    # in order, own half, not swapped: no leg over the middle (the homes are in each robot's own half)
    m, prog, orders = _orders(school, "e1")
    assert all(v == 0 for v in prog["production"]["experiment"]["crossings"].values())


def test_zigzag_alternates_zones(school):
    m, prog, orders = _orders(school, "e3")
    _p, zones, _w = SP.points_cm(school)
    own = [l for l in orders["Alpha"] if l not in prog["production"]["experiment"]["traded"]["Alpha"]]
    zs = [zones[l] for l in own]
    switches = sum(1 for a, b in zip(zs, zs[1:]) if a != b)
    assert switches >= len(zs) - 3          # nearly every step changes zone


def test_swap_exchanges_the_halves_homes_stay(school):
    m0, _p, o0 = _orders(school, "e1", False)
    m1, _p, o1 = _orders(school, "e1", True)
    assert set(o1["Alpha"]) == set(o0["Bravo"]) and set(o1["Bravo"]) == set(o0["Alpha"])
    assert [r["start"] for r in m0["robots"]] == [r["start"] for r in m1["robots"]]


def test_shuffle_is_fixed_by_its_seed_and_recorded(school):
    _m, p1, o1 = _orders(school, "e4")
    _m, p2, o2 = _orders(school, "e4")
    assert o1 == o2 and p1["production"]["experiment"]["seed"] == EX.ORDER_SEED
    assert o1["Alpha"] != sorted(o1["Alpha"], key=EX.num)


def test_experiments_differ_from_each_other(school):
    seen = set()
    for exp in ("e1", "e2", "e3", "e4"):
        _m, _p, o = _orders(school, exp)
        seen.add(tuple(o["Alpha"]))
    assert len(seen) == 4


def test_swap_alternates_with_the_run_counter(tmp_path, school):
    site = dict(school, conf_dir=str(tmp_path))
    got = []
    for k in range(4):
        sw, _words = EX.swap_for(site, "auto")
        got.append(sw)
        EX.counter_add(site, {"experiment": "e1", "swap_halves": sw})
    assert got == [False, True, False, True]
    assert EX.swap_for(site, "on")[0] is True and EX.swap_for(site, "off")[0] is False
    assert EX.counter_read(site)["runs"] == 4


def test_old_default_orders_kept_as_an_option(school):
    _m, _w, prog, orders = PR.site_production(school)          # no experiment: as before
    assert "experiment" not in prog["production"]
    _m, _p, od = _orders(school, "default")
    assert od == orders


# ------------------------------------------------------------------------------------------------ the sync wiggle

def test_wiggle_block_both_robots_at_the_start(school):
    m, prog, _o = _orders(school, "e1")
    assert prog["blocks"][0]["block"] == "sync-wiggle" and prog["blocks"][0]["robots"] == ["Alpha", "Bravo"]
    MS.build(m, prog)
    # home's site: no experiments block, so no wiggle
    home = SI.load("home")
    assert not EX.wiggle_on(home) and EX.wiggle_on(school)
    _m, _w, p_off, _o = PR.site_production(school, experiment="e1", wiggle=False)
    assert p_off["blocks"][0]["block"] != "sync-wiggle"


def test_wiggle_plans_quarter_turn_and_back_at_the_same_moment(school):
    import planner as P
    m, prog, _o = _orders(school, "e1")
    mm = MS.build(m, prog)
    plan = MS.plan_synced(mm, over={"separation_cm": 8.0}, t_limit_s=60.0)
    for n in ("Alpha", "Bravo"):
        st = [s for s in plan["robots"][n]["steps"] if str(s.get("label", "")).startswith("sync wiggle")]
        labs = [s["label"] for s in st]
        assert labs[0] == "sync wiggle 1 turn" and "sync wiggle 3 turn back" in labs
        t1 = [s for s in st if s["label"] == "sync wiggle 1 turn"][0]
        assert t1["t_start"] == 0.0 and abs(abs(t1["turn_deg"]) - 90.0) < 1.0
        back = [s for s in st if s["label"] == "sync wiggle 3 turn back"][0]
        assert abs(back["to"][2] - plan["robots"][n]["start"][2]) % 360 < 1.0
    _ = P


def test_wiggle_events_from_the_step_log(tmp_path):
    p = tmp_path / "coordinator.jsonl"
    ev = []
    for n, t0 in (("Alpha", 100.0), ("Bravo", 100.02)):
        for k, lab in enumerate(["sync wiggle 1 turn", "sync wiggle 2 stop", "sync wiggle 3 turn back",
                                 "sync wiggle 4 stop"]):
            ev.append({"event": "labelled_step", "robot": n, "i": k, "label": lab, "state": "started", "t": t0 + k})
            ev.append({"event": "labelled_step", "robot": n, "i": k, "label": lab, "state": "done", "t": t0 + k + 0.9})
    ev.append({"event": "labelled_step", "robot": "Alpha", "i": 9, "label": "point P01", "state": "done", "t": 200})
    p.write_text("\n".join(json.dumps(e) for e in ev))
    w = EX.wiggle_events(str(p))
    assert w["Alpha"]["start"] == 100.0 and abs(w["Alpha"]["end"] - 103.9) < 1e-9 and len(w["Bravo"]["parts"]) == 4


# ------------------------------------------------------------------------------------------------ run exports

def _fake_mission(tmp_path, fixes_jump=False):
    """A tiny mission folder: one robot drives 50 cm along +x at 10 cm/s; the camera sees it every 0.1 s; its own
    fused pose is pulled by a camera fix at 2.5 s (a 3 cm jump in follow.csv)."""
    m = tmp_path / "mission"
    (m / "run").mkdir(parents=True)
    (m / "bridge").mkdir()
    t0, off = 1000.0, 900.0                  # Pi time = ms / 1000 + 900
    with open(m / "run" / "clock_sync.csv", "w") as f:
        f.write("robot,boot,token,t_send,t_recv,robot_ms,rtt_ms,offset_s\nAlpha,AB12,c1,0,0,0,10,%.4f\n" % off)
    with open(m / "run" / "param.csv", "w") as f:
        f.write("robot,t_recv,run,name,value\nAlpha,%.3f,AB12-1,wheel_scale,1.0\nAlpha,%.3f,AB12-1,gyro_scale,1.0\n"
                % (t0 - 1, t0 - 1))
    with open(m / "run" / "runinfo.csv", "w") as f:
        f.write("robot,t_recv,run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,yaw_sign,"
                "odo_sign\nAlpha,%.3f,AB12-1,follow,x,x,0,90,0,-50,,1,1\n" % (t0 - 1))
    cols = ["robot", "t_recv", "run", "ms", "step", "wl_deg", "wr_deg", "x_cm", "y_cm", "th_deg", "yaw_deg", "gz"]
    with open(m / "run" / "raw.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for k in range(0, 251):               # 5 s at 50 a second
            ms = 100000 + 20 * k
            d = min(50.0, 10.0 * k * 0.02)
            w.writerow(["Alpha", ms / 1000.0 + off + 0.01, "AB12-1", ms, 0, d / 0.0593, d / 0.0593, "%.2f" % d, 0, 0,
                        "%.2f" % 37.0, 0])
    with open(m / "run" / "follow.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "ms", "step_id", "state", "x_cm", "y_cm", "th_deg"])
        for k in range(0, 251, 5):
            ms = 100000 + 20 * k
            d = min(50.0, 10.0 * k * 0.02)
            jump = 3.0 if (fixes_jump and ms >= 102500) else 0.0
            w.writerow(["Alpha", ms / 1000.0 + off, "AB12-1", ms, 1, "run", 20.0 + d + jump, 30.0, 0.0])
    with open(m / "bridge" / "tag_poses.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cam", "robot", "id", "x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio",
                    "px_per_cm", "u", "v", "used"])
        for k in range(50):
            t = 100.0 + off + 0.1 * k
            w.writerow([t, "overhead", "Alpha", 0, 20 + k, 30.0, 0.0, 0.1, 1.0, 1, 4, 0, 0, "yes"])
            if k % 2 == 0:
                w.writerow([t + 0.01, "side", "Alpha", 0, 20.2 + k, 30.0, 0.0, 0.1, 1.0, 1, 4, 0, 0, "yes"])
            if k == 10:
                w.writerow([t + 0.02, "side", "Alpha", 0, 90, 90, 0.0, 0.1, 1.0, 1, 4, 0, 0, "size off"])
    with open(m / "mission_summary.json", "w") as f:
        json.dump({"sync_wiggle": {"Alpha": {"start": 1000.0, "end": 1004.0, "parts": []}}}, f)
    with open(m / "mission_cmd.json", "w") as f:
        json.dump({"args": {"site": "school-conference"}}, f)
    return str(m)


def test_camera_track_every_read_fused_refused_left_out(tmp_path):
    m = _fake_mission(tmp_path)
    p, c = RX.camera_track(m)
    rows = list(csv.DictReader(open(p)))
    assert c["Alpha"] == 50 == len(rows)                # 50 moments; the side camera's reads fused into 25 of them
    assert sum(1 for r in rows if r["cams"] == "overhead+side") == 25
    assert all(float(r["x_cm"]) < 80 for r in rows)      # the refused read (90, 90) left out
    both = [r for r in rows if r["n_reads"] == "2"][0]
    assert abs(float(both["cams_apart_cm"]) - 0.2) < 1e-6 and float(both["sigma_cm"]) < 0.1


def test_odometry_raw_has_no_camera_correction(tmp_path):
    a = _fake_mission(tmp_path / "a", fixes_jump=False)
    b = _fake_mission(tmp_path / "b", fixes_jump=True)    # the fused pose jumps 3 cm at a camera fix
    pa, _c, _w = RX.odometry_raw(a)
    pb, _c, _w = RX.odometry_raw(b)
    ra = list(csv.DictReader(open(pa)))
    rb = list(csv.DictReader(open(pb)))
    assert [r["x_odo_cm"] for r in ra] == [r["x_odo_cm"] for r in rb]      # the fix changes nothing in it
    xs = [float(r["x_odo_cm"]) for r in rb]
    steps = [b_ - a_ for a_, b_ in zip(xs, xs[1:])]
    assert max(steps) < 0.25                                               # no jump: 10 cm/s x 20 ms = 0.2 cm
    assert abs(xs[0] - 20.0) < 1e-6 and abs(xs[-1] - 70.0) < 0.05          # anchored once at the start pose
    assert abs(float(rb[100]["t"]) - (102.0 + 900.0)) < 1e-3               # robot clock on the Pi clock


def test_bundle_has_our_files_and_nothing_from_rfid(tmp_path):
    m = _fake_mission(tmp_path)
    open(os.path.join(m, "rfid_capture.csv"), "w").write("x")
    os.makedirs(os.path.join(m, "rfid"), exist_ok=True)
    open(os.path.join(m, "rfid", "labelled.csv"), "w").write("x")
    p, names = RX.bundle(m)
    assert p.endswith(".zip") and os.path.dirname(p) == m
    z = zipfile.ZipFile(p)
    allz = z.namelist()
    assert not any("rfid" in n.lower() for n in allz)
    for need in ("run/camera_track.csv", "run/odometry_raw.csv", "run/raw.csv", "run/follow.csv",
                 "mission_summary.json", "README.txt", "site/school-conference.json",
                 "site/school-conference-points.csv"):
        assert need in names, need
    rd = z.read([n for n in allz if n.endswith("README.txt")][0]).decode()
    assert "Alpha 1000.000 -> 1004.000" in rd and "camera_track.csv" in rd


def test_repo_ignores_logs_so_bundles_never_commit():
    gi = open(os.path.join(os.path.dirname(os.path.dirname(FLEET)), ".gitignore")).read()
    assert "logs" in gi


# ------------------------------------------------------------------------------------------------ duplicated points

def _coord():
    import test_keep_going as KG
    mm, plan = KG.production_plan()
    return KG.coord(mm, plan)


def test_a_point_handed_back_and_forth_is_listed_once():
    c = _coord()
    c.args.dedupe_points = "on"
    # P2 went Bravo -> Alpha, then back Alpha -> Bravo (newest); Bravo has it now, Alpha does not
    item = {"goto": [320.0, 120.0], "label": "point 2"}
    c.reassigned = [{"point": "point 2", "items": [item], "frm": "Bravo", "to": "Alpha"},
                    {"point": "point 2", "items": [item], "frm": "Alpha", "to": "Bravo"}]
    c.plan["robots"]["Alpha"]["visit_order"] = [v for v in c.plan["robots"]["Alpha"]["visit_order"]
                                                if v.get("label") not in ("point 2", "at point 2")]
    c.keep_reassigned()
    labs = [v.get("label") for n in ("Alpha", "Bravo") for v in c.plan["robots"][n]["visit_order"]]
    assert labs.count("point 2") == 1
    # the old way (switch off) brings the bug back: the stale record puts it in Alpha's list again
    c2 = _coord()
    c2.args.dedupe_points = "off"
    c2.reassigned = list(c.reassigned)
    c2.plan["robots"]["Alpha"]["visit_order"] = [v for v in c2.plan["robots"]["Alpha"]["visit_order"]
                                                 if v.get("label") not in ("point 2", "at point 2")]
    c2.keep_reassigned()
    labs2 = [v.get("label") for n in ("Alpha", "Bravo") for v in c2.plan["robots"][n]["visit_order"]]
    assert labs2.count("point 2") == 2


def test_insert_never_lists_a_point_twice_or_a_done_point():
    c = _coord()
    vo = c.plan["robots"]["Alpha"]["visit_order"]
    have = [v for v in vo if v.get("label") == "point 1"]
    c.insert_visits("Alpha", have + [{"wait_s": 3, "label": "at point 1"}])
    assert [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]].count("point 1") == 1
    c.done_labels = {"at point 4"}
    c.insert_visits("Alpha", [{"goto": [280.0, 50.0], "label": "point 4"}, {"wait_s": 3, "label": "at point 4"}])
    assert "point 4" not in [v.get("label") for v in c.plan["robots"]["Alpha"]["visit_order"]]


def test_both_held_loop_is_capped_and_switchable(monkeypatch):
    import run_plan as RP
    monkeypatch.setattr(RP, "HELD_NEVER_END", False)     # (e16) the cap is the switch now (MIRLT_HELD_NEVER_END=0)
    c = _coord()
    c.all_held_n = RP.Coordinator.ALL_HELD_MAX - 1
    c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
    c.retire(c.bots["Alpha"], "nothing heard", kind="silent")
    assert c.abort and "every robot held 3 times" in c.abort
    c = _coord()
    c.args.all_held_max, c.args.all_held_max_s = 0, 0
    c.all_held_n = 10
    c.retire(c.bots["Bravo"], "nothing heard", kind="silent")
    c.retire(c.bots["Alpha"], "nothing heard", kind="silent")
    assert not c.abort and any("every robot is held" in s for s in c.said)
