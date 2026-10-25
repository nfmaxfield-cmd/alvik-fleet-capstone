"""tools/raw_units.py: the RAW speed columns' units (v_cms is mm/s; rpm, v_cms, w_dps read 1.08 high)."""
import json
import math
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import raw_units as RU  # noqa: E402
import check_session as cs  # noqa: E402


def test_real_robot_ratios():
    # what the Oct 25 logs show: v_cms / mean rpm = 1.78 (= 10 x the cm/s factor of a 1.7 cm wheel)
    c = RU.robot_columns(30.0, 30.0)
    assert c["v_cms"] / c["wl_rpm"] == pytest.approx(10 * 2 * math.pi * 1.7 / 60, rel=1e-9)
    assert c["wl_rpm"] == pytest.approx(30 * 1.08)
    assert RU.wheel_rpm(c["wl_rpm"]) == pytest.approx(30.0)
    assert RU.speed_cms(c["v_cms"]) == pytest.approx(30 * 2 * math.pi * 1.7 / 60)
    c = RU.robot_columns(-10.0, 10.0, wheel_track_cm=9.0)
    true_w = math.degrees(20 * 2 * math.pi * 1.7 / 60 / 9.0)
    assert RU.turn_dps(c["w_dps"]) == pytest.approx(true_w)
    assert RU.speed_cms(c["v_cms"]) == pytest.approx(0.0, abs=1e-12)


def test_missing_values_are_nan():
    for f in (RU.speed_cms, RU.wheel_rpm, RU.turn_dps):
        assert math.isnan(f(None)) and math.isnan(f("")) and math.isnan(f("x"))
    assert RU.speed_cms("108.0") == pytest.approx(10.0)


def _write(folder, rows, session=None, sim_info=None, sim_info_where="up_sim"):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "raw.csv"), "w") as f:
        f.write("robot,t_recv,run,ms,wl_rpm,wr_rpm,v_cms,w_dps\n")
        for r in rows:
            f.write("Left,1,a-1,%d,%s,%s,%s,%s\n" % (r[0], r[1], r[2], r[3], r[4]))
    if session is not None:
        json.dump(session, open(os.path.join(folder, "session.json"), "w"))
    if sim_info is not None:
        d = os.path.join(os.path.dirname(folder), "sim") if sim_info_where == "up_sim" else os.path.dirname(folder)
        os.makedirs(d, exist_ok=True)
        json.dump(sim_info, open(os.path.join(d, "sim_info.json"), "w"))


def test_legacy_simulator_logs_rescaled(tmp_path):
    rows = [(0, "10.00", "10.00", "1.78", "0.00")]          # old simulator: v_cms in cm/s, no 1.08
    real = tmp_path / "real" / "robot-logs-1"
    _write(str(real), rows, session={"laptop": "x"})
    assert not RU.legacy_sim_folder(str(real))
    assert cs.rows(str(real), "raw")[0]["v_cms"] == "1.78"   # real logs are never touched

    old_dry = tmp_path / "dry" / "robot-logs-1"               # e02 dry run: ../sim/sim_info.json, no mark
    _write(str(old_dry), rows, session={"laptop": "x"}, sim_info={"note": "x"})
    assert RU.legacy_sim_folder(str(old_dry))
    r = cs.rows(str(old_dry), "raw")[0]
    assert float(r["v_cms"]) == pytest.approx(1.78 * 10.8, abs=1e-3)
    assert float(r["wl_rpm"]) == pytest.approx(10.8, abs=1e-3)
    assert RU.speed_cms(r["v_cms"]) == pytest.approx(1.78, abs=1e-3)

    old_sim = tmp_path / "simold" / "session"                 # tools/sim/simulate.py: session.json simulated
    _write(str(old_sim), rows, session={"simulated": True}, sim_info={"seed": 7}, sim_info_where="up")
    assert RU.legacy_sim_folder(str(old_sim))

    new_sim = tmp_path / "simnew" / "session"
    _write(str(new_sim), rows, session={"simulated": True, "raw_units": RU.RAW_UNITS},
           sim_info={"seed": 7, "raw_units": RU.RAW_UNITS}, sim_info_where="up")
    assert not RU.legacy_sim_folder(str(new_sim))
    assert cs.rows(str(new_sim), "raw")[0]["v_cms"] == "1.78"


def test_session_json_marks_new_simulator_logs(tmp_path):
    sys.path.insert(0, os.path.join(TOOLS, "sim"))
    import network
    network.write_session_json(str(tmp_path), 0.0, "2026-10-25T00:00:00")
    d = json.load(open(tmp_path / "session.json"))
    assert d["simulated"] and d["raw_units"] == RU.RAW_UNITS
