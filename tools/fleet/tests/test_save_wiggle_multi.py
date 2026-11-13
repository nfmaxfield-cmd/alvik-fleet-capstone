"""Build e03 (5 Nov 2026): calibration saved on the robot (firmware `save` / `unsave`, `corr save` on older firmware),
wiggle-first naming for every run, every robot found calibrated in turn (the other parked and kept out of the way),
waking any robot from any address (robot_sleep.py wake any / --name). Simulator only: no robot, no Pi."""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "recognition"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)


def _harness(tmp_path, specs, rs_args, timeout=200):
    cmd = ["unshare", "-rn", sys.executable, os.path.join(HERE, "sim_wake_harness.py"), str(tmp_path)] + specs + \
          ["--"] + rs_args
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO, timeout=timeout)
    return out.stdout + out.stderr


# ------------------------------------------------------------------------------------------------- waking
def test_wake_any_wakes_the_robot_from_whatever_address_it_checks_in_from(tmp_path):
    # the broadcast goes nowhere (127.0.0.9): the robot is only heard at its check-in, and woken at that address
    out = _harness(tmp_path, ["AL-1DC0:0.3:0.3"], ["wake", "any", "--broadcast", "127.0.0.9", "--max-min", "1.5"])
    assert "AL-1DC0: awake at 127.0.0.2, battery" in out and "RC=0" in out, out


def test_wake_by_name_leaves_the_other_robot_alone(tmp_path):
    out = _harness(tmp_path, ["AL-1DC0:0.2:0.1", "AL-2B7F:0.3:0.5"],
                   ["wake", "--name", "AL-2B7F", "--broadcast", "127.0.0.9", "--max-min", "1.5"])
    assert "AL-2B7F: awake at 127.0.0.3" in out and "AL-1DC0: awake" not in out and "RC=0" in out, out


def test_wake_by_address_still_works_and_names_robots_heard_elsewhere(tmp_path):
    out = _harness(tmp_path, ["AL-1DC0:0.2:0.1"], ["wake", "127.0.0.50", "--max-min", "0.3"])
    assert "heard AL-1DC0 at 127.0.0.2" in out and "wake --name AL-1DC0" in out and "RC=1" in out, out
    out = _harness(tmp_path / "b", ["AL-1DC0:0.2:0.1"], ["wake", "127.0.0.2", "--max-min", "1"])
    assert "127.0.0.2: awake" in out and "RC=0" in out, out


# ------------------------------------------------------------------------------------------------- the robot's save
def test_sim_robot_save_survives_sleep_and_old_firmware_has_corr_only(tmp_path):
    out = _harness(tmp_path, ["AL-1DC0:0.05:0.99"], ["cmds", "127.0.0.2", "wait:4", "wake", "set wheel_scale 0.97", "save",
                                                     "set wheel_scale 0.5", "sleep 1", "wait:68", "wake",
                                                     "get wheel_scale", "unsave", "get wheel_scale"], timeout=150)
    assert "saved on the robot (kept after power-off): wheel_scale 0.9700" in out, out
    assert "calibration at power-on: wheel_scale 0.9700 (saved)" in out          # the wake from sleep is a restart
    assert out.count("wheel_scale = 0.970") == 2 and "back to defaults wheel_scale 1.0000" in out
    assert out.rstrip().endswith("RC=0") and "wheel_scale = 1.000" in out
    out = _harness(tmp_path / "old", ["AL-1DC0:0.05:0.99", "--fw=AL-1DC0=2026-10-29b"],
                   ["cmds", "127.0.0.2", "wait:4", "wake", "save", "corr save wheel_scale 0.97", "get wheel_scale",
                    "id"])
    assert "unknown command 'save'" in out and "corrections (saved by hand)" in out and "wheel_scale = 0.970" in out
    assert "unknown command 'id'" in out


def test_firmware_save_commands_and_version():
    ino = open(os.path.join(REPO, "firmware", "home_lab", "home_lab.ino")).read()
    corr = open(os.path.join(REPO, "firmware", "home_lab", "corr.h")).read()
    assert '#define FW_VERSION "2026-11-' in ino and "2026-11-05a (Nick, 5 Nov" in ino    # 11-05b (free roam) kept them
    for v in ('verb == "save"', 'verb == "unsave"', 'verb == "defaults"', 'verb == "saved"', 'verb == "id"'):
        assert v in ino
    assert "prefs.isKey" in corr and "prefs.clear()" in corr and 'showSaved("at power-on")' in corr
    r = subprocess.run(["sh", os.path.join(TOOLS, "fwcheck", "check.sh")], capture_output=True, text=True)
    assert r.returncode == 0 and not r.stdout.strip(), r.stdout


# ------------------------------------------------------------------------------------------------- slots and names
def test_assign_slots_by_position_pins_and_names():
    import wiggle_bind as WB
    homes = {"Alpha": (150, 82), "Bravo": (250, 82)}
    seen = {"AL-1DC0": (240, 80), "AL-2B7F": (140, 90)}
    assert WB.assign_slots(["Alpha", "Bravo"], ["AL-1DC0", "AL-2B7F"], seen, homes) == {"AL-1DC0": "Bravo",
                                                                                        "AL-2B7F": "Alpha"}
    # pinned in the site (robots[].wifi) wins over position
    assert WB.assign_slots(["Alpha", "Bravo"], ["AL-1DC0", "AL-2B7F"], seen, homes, pins={"AL-1DC0": "Alpha"}) == \
        {"AL-1DC0": "Alpha", "AL-2B7F": "Bravo"}
    # slot_by name: a robot already called by a slot keeps it
    assert WB.assign_slots(["Alpha", "Bravo"], ["Bravo", "AL-1DC0"], {"Bravo": (150, 82), "AL-1DC0": (250, 80)},
                           homes, by="name") == {"Bravo": "Bravo", "AL-1DC0": "Alpha"}
    with pytest.raises(Exception) as e:
        WB.assign_slots(["Alpha"], ["AL-1", "AL-2"], {}, homes)
    assert "2 robots answered" in str(e.value)


def test_rename_steps_never_give_two_robots_one_name():
    import wiggle_bind as WB
    cur = {"Alpha": "192.168.1.10", "Bravo": "192.168.1.11"}
    steps = WB.rename_steps(cur, {"Alpha": "Bravo", "Bravo": "Alpha"})
    names = dict((ip, n) for n, ip in cur.items())
    for ip, old, new in steps:
        assert names[ip] == old
        names[ip] = new
        assert len(set(names.values())) == len(names)       # never two robots with one name
    assert names == {"192.168.1.10": "Bravo", "192.168.1.11": "Alpha"}
    assert WB.rename_steps({"Alpha": "x"}, {"Alpha": "Alpha"}) == []
    assert WB.rename_steps({"AL-1DC0": "1.2.3.4"}, {"AL-1DC0": "Alpha"}) == [("1.2.3.4", "AL-1DC0", "Alpha")]


def test_wiggle_bind_in_the_simulator_binds_and_renames(tmp_path):
    out = subprocess.run(["unshare", "-rn", sys.executable, os.path.join(FLEET, "wiggle_bind.py"), "--site",
                          "school-conference", "--robots", "Alpha,Bravo", "--dry", "--session", str(tmp_path),
                          "--sim", "AL-1DC0:2:P14", "--sim", "AL-2B7F:3:P05", "--rename", "on"], capture_output=True,
                         text=True,
                         cwd=REPO, timeout=240)
    rec = json.load(open(tmp_path / "binding.json"))
    assert out.returncode == 0, out.stdout + out.stderr
    assert sorted(rec["slots"].values()) == [2, 3] and sorted(rec["slots"]) == ["Alpha", "Bravo"]
    assert {r["to"] for r in rec["renamed"]} == {"Alpha", "Bravo"}
    assert "is now called" in out.stdout


def test_mission_run_wiggle_first_switch_and_children(tmp_path, monkeypatch):
    import argparse
    import mission_run as MR
    import sites as SI
    import wiggle_bind as WB
    site = SI.load("school-conference")
    a = argparse.Namespace(site_=site, binding="auto", dry=False, robots="Alpha,Bravo", stream="127.0.0.1:8000",
                           calib="", robot=[], tags_=dict(site["tags"]), binding_file="")
    rec = {"slots": {"Alpha": 3, "Bravo": 2}, "empty_slots": [], "robots": [], "renamed": []}
    monkeypatch.setattr(WB, "run_for_mission", lambda *x, **k: rec)
    assert MR.wiggle_first(a, str(tmp_path)) == 0
    assert a.tags_["ids"] == {"Alpha": 3, "Bravo": 2} and a.tags_["ids_typed"] == {"Alpha": 0, "Bravo": 1}
    a2 = argparse.Namespace(**dict(vars(a), tags_=dict(site["tags"]), binding="fixed"))
    assert MR.wiggle_first(a2, str(tmp_path)) == 0 and a2.tags_["ids"] == {"Alpha": 0, "Bravo": 1}   # the old way
    rec2 = dict(rec, empty_slots=["Bravo"], slots={"Alpha": 3})
    monkeypatch.setattr(WB, "run_for_mission", lambda *x, **k: rec2)
    a3 = argparse.Namespace(**dict(vars(a), tags_=dict(site["tags"])))
    assert MR.wiggle_first(a3, str(tmp_path)) == 5                       # a slot with no robot: not started
    p = tmp_path / "b.json"
    p.write_text(json.dumps(rec))
    a4 = argparse.Namespace(**dict(vars(a), tags_=dict(site["tags"]), binding_file=str(p)))
    monkeypatch.setattr(WB, "run_for_mission", lambda *x, **k: pytest.fail("a feel-out child must not wiggle again"))
    assert MR.wiggle_first(a4, str(tmp_path)) == 0 and a4.tags_["ids"] == {"Alpha": 3, "Bravo": 2}


# ------------------------------------------------------------------------------------------------- several robots
def test_parked_robot_keepout_and_detour_rule():
    import calib_run as CR
    import sites as SI
    S = CR.Site(SI.load("school-conference"), "P05")
    assert S.park_label == "P05"
    S.set_parked({"AL-1DC0": S.points["P11"]})
    assert S.parked_near(S.points["P11"])[0] == "parked robot AL-1DC0"
    assert S.parked_near(S.points["P14"]) is None
    probs = S.problems((S.points["P11"][0] - 40, S.points["P11"][1]), (S.points["P11"][0] + 40, S.points["P11"][1]))
    assert probs and CR.solid(probs[0])                        # a leg through it: a detour, as round an outlet
    S.set_parked({})
    assert not [k for k in S.keepouts if k.get("kind") == "robot"]
    assert CR.PARK_SPOTS[:2] == ("P11", "P01") and CR.PARK_SPOTS_E03[:2] == ("P11", "P05")   # P01 since 5 Nov 11:00
    # the spots: inside the camera's view, clear of the edge margin and every keep-out
    for p in CR.PARK_SPOTS:
        x, y = S.points[p]
        assert S.edge_dist((x, y)) >= S.edge and S.keepout_dist((x, y))[0] > CR.KEEPOUT_CLEAR_CM and x < S.view_x
