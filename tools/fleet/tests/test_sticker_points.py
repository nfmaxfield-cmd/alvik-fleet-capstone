"""School sticker points and runs with or without tags (2026-11-03): Jay's sheet imported as typed points, the P09/P10
overrides, the 13 cm edge room, homes at P11/P22 done where the robot stands, the raised box kept out, zone split and
short orders, the run-tracking switch (tags / lights / none) and its words. Plans only; no robot, no Pi."""
import csv
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)

import sites as SI              # noqa: E402
import sticker_points as SP     # noqa: E402
import production as PR         # noqa: E402

IN = 2.54


@pytest.fixture(scope="module")
def school():
    return SI.load("school-conference")


def test_points_file_is_the_sheet_as_typed(school):
    rows = SP.read_csv(SP.points_file(school))
    assert len(rows) == 40 and rows[0]["id"] == "P01"
    by = {r["id"]: r for r in rows}
    # the file keeps Jay's values; the corrections live in the site file
    assert (by["P09"]["x_in"], by["P09"]["y_in"]) == (17, 37.5)
    assert (by["P10"]["x_in"], by["P10"]["y_in"]) == (33, 27.5)
    assert sum(1 for r in rows if r["zone"] == 1) == 10 and sum(1 for r in rows if r["zone"] == 4) == 10


def test_overrides_applied_and_said(school):
    pts, zones, words = SP.points_cm(school)
    assert pts["P09"] == [round(17 * IN, 2), round(57.5 * IN, 2)]
    assert pts["P10"] == [round(33 * IN, 2), round(37.5 * IN, 2)]
    assert len(words) == 2 and all("confirmed by Jay-Sun" in w for w in words)   # status updated 3 Nov (was "assumed, to confirm")
    # every zone matches its x (zone lines 39 / 78 / 117 in)
    assert all(SP.zone_of(xy[0]) == zones[k] for k, xy in pts.items())


def test_xlsx_and_csv_readers_agree(tmp_path):
    openpyxl = pytest.importorskip("openpyxl")
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = "Points"
    ws.append(["Conference table: 40 test points"])
    ws.append(["Point ID", "x (in)", "y (in)", "Zone (1-4)", "Notes"])
    ws.append(["Example", 20.5, 30.25, 1, "example only"])
    ws.append(["P01", 12, 12.5, 1, None])
    ws.append(["P02", 29.75, 21.25, 1, None])
    p = tmp_path / "t.xlsx"
    wb.save(p)
    rows = SP.read_xlsx(str(p))
    assert [r["id"] for r in rows] == ["P01", "P02"]
    c = tmp_path / "t.csv"
    c.write_text("1,12,12.5,1\n2,29.75,21.25\n")
    rows2 = SP.read_csv(str(c))
    assert [(r["id"], r["x_in"], r["y_in"]) for r in rows2] == [(r["id"], r["x_in"], r["y_in"]) for r in rows]
    assert rows2[1]["zone"] is None


def test_edge_room_13_reaches_every_sticker_home_stays_17(school):
    # (e08, A2) the one derived rule: roof tags (school) 6.72 + 1.7 + 2.4 = 10.8; lights (home) + 5 = 15.8. Fixed: 13 / 17
    import safety_rules as SR
    assert PR.site_edge_cm(school) == (13.0 if SR.EDGE_RULE == "fixed" else 10.8)
    assert PR.site_edge_cm(SI.load("home")) == (17.0 if SR.EDGE_RULE == "fixed" else 15.8)
    m, words, prog, orders = PR.site_production(school)
    assert not [w for w in words if "open edge" in w]           # no sticker moved in from an edge at 13 cm
    pts, _, _ = SP.points_cm(school)
    near = min(min(x, y, 396.24 - x, 167.64 - y) for x, y in pts.values())
    assert 13.0 <= near < 13.5                                   # P04 at 13.3 cm


def test_homes_at_p11_p22_facing_plus_y_and_done_first(school):
    m, words, prog, orders = PR.site_production(school)
    st = {r["name"]: r["start"] for r in m["robots"]}
    assert st["Alpha"][:2] == [round(59 * IN, 2), round(32.5 * IN, 2)] and st["Alpha"][2] == 90.0
    assert st["Bravo"][:2] == [round(96 * IN, 2), round(32.5 * IN, 2)]
    assert orders["Alpha"][0] == "P11" and orders["Bravo"][0] == "P22"
    first = [b for b in prog["blocks"] if b.get("robot") == "Alpha" or b.get("robots") == ["Alpha"]][0]
    assert first["block"] == "wait" and first["label"] == "at point P11"   # stood at, not driven to
    assert not any(b.get("block") == "go-to-spot" and b.get("spot") == "P11" for b in prog["blocks"])


def test_zone_split_all_40_short_orders(school):
    m, words, prog, orders = PR.site_production(school)
    a, b = set(orders["Alpha"]), set(orders["Bravo"])
    assert len(a) == 20 and len(b) == 20 and not a & b
    assert all(m["spots"][k][0] < 78 * IN for k in a) and all(m["spots"][k][0] > 78 * IN for k in b)
    # the 2-opt order is no longer than the order by number
    for n, labs in orders.items():
        h = m["robots"][[r["name"] for r in m["robots"]].index(n)]["start"]
        assert PR.tour_len(h, labs[1:], m["spots"]) <= PR.tour_len(h, sorted(labs[1:]), m["spots"]) + 1e-6


def test_typed_orders_override_zones(school):
    s = json.loads(json.dumps(school))
    s["production"]["orders"] = {"Alpha": ["P11", "P01"], "Bravo": ["P22", "P40"]}
    _, _, prog, orders = PR.site_production(s)
    assert orders == {"Alpha": ["P11", "P01"], "Bravo": ["P22", "P40"]}


def test_raised_box_kept_out_and_homes_clear(school):
    # Jay-Sun 3 Nov: three measured outlets replace the assumed 12 x 12 in box (kept, switched off)
    ko = PR.site_keepouts(school)
    assert [k["name"] for k in ko] == ["middle outlet box", "zone 1 outlet plate", "zone 4 outlet plate"]
    assert all(k["solid"] for k in ko)
    xs = [p[0] for p in ko[0]["polygon"]]
    assert abs(min(xs) - 74.81 * IN) < 0.01 and abs(max(xs) - 81.25 * IN) < 0.01
    m, words, prog, orders = PR.site_production(school)
    assert any(k["name"] == "middle outlet box" for k in m["keepouts"])
    # homes well clear of every outlet; no sticker needs moving (nearest: P12, 16.0 cm from the middle box)
    for r in m["robots"]:
        for k in ko:
            q, inside = PR._nearest_on_poly(__import__("numpy").array(r["start"][:2]), k["polygon"])
            assert not inside and math.hypot(r["start"][0] - q[0], r["start"][1] - q[1]) > 25
    # (2026-11-16) the line saying how much room the paths keep from the keep-outs names them too: not a moved sticker
    assert not any("outlet" in w and not w.startswith("keep-outs (") for w in words)
    s = json.loads(json.dumps(school))
    s["keepouts"][0]["use"] = True                                # the old assumed box, switched back on
    assert any(k["name"] == "raised box" for k in PR.site_keepouts(s))
    for k in s["keepouts"]:
        k["use"] = False                                          # the switch
    assert PR.site_keepouts(s) == []


def test_switches_keep_the_old_ways(school):
    s = json.loads(json.dumps(school))
    s["points"]["use"] = "clicked"
    assert not SP.typed_on(s)
    s["homes"]["use"] = "middle"
    assert PR.site_home_at(s, {}) is None
    # the old middle homes unchanged
    hs = PR.homes(396.24, 167.64, ["Alpha", "Bravo"])
    assert hs["Alpha"][0] == round(396.24 / 2 - 15, 2)


def test_flag_seen_sticker_never_moves():
    w = SP.flag_seen({"P01": [30.0, 30.0]}, {"P01": [36.0, 30.0], "P02": [1, 1]})
    assert len(w) == 1 and "P01" in w[0] and "6.0 cm" in w[0]
    assert SP.flag_seen({"P01": [30.0, 30.0]}, {"P01": [31.0, 30.0]}) == []


def test_run_tracking_switch_and_validation(school, tmp_path, monkeypatch):
    assert school["run_tracking"] == "tags" and SI.load("home")["run_tracking"] == "lights"
    assert "during a run" in SI.words(school)
    monkeypatch.setattr(SI, "SITES_DIR", str(tmp_path))
    (tmp_path / "bad.json").write_text(json.dumps({"robots": [], "run_tracking": "radar"}))
    with pytest.raises(SystemExit):
        SI.load("bad")


def test_mission_run_none_mode_plans_from_the_site(tmp_path):
    import subprocess
    out = tmp_path / "plan"
    r = subprocess.run([sys.executable, os.path.join(FLEET, "mission_run.py"), "--site", "school-conference", "--dry",
                        "--plan-only", "--tracking", "none", "--session", str(out)], capture_output=True, text=True,
                       timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    prog = json.load(open(out / "program.json"))
    assert set(prog["robots"]) == {"Alpha", "Bravo"}
    plan = json.load(open(out / "plan.json"))
    assert not [w for w in plan.get("warnings") or [] if "skips task" in str(w)]     # all 40 reached
    cmd = json.load(open(out / "mission_cmd.json"))
    assert cmd["args"]["tracking_"] == "none"


def test_mission_control_loads_the_default_and_keeps_it_after_a_restart(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")
    import mission_control as MC
    s1 = MC.Session(str(tmp_path / "mc"))
    st = s1.state()
    assert st["site"]["name"] == "school-conference" and st["site"]["points_typed"]
    assert {k: len(v) for k, v in st["production_orders"].items()} == {"Alpha": 20, "Bravo": 20}
    assert len(st["map"]["spots"]) == 40 and st["tracking_now"] == "tags"
    assert s1.preview is not None                      # planned, ready to run
    s2 = MC.Session(str(tmp_path / "mc"))              # a restart: the same orders, not the first-3 default
    assert s2.state()["production_orders"] == st["production_orders"]
    s2.settings["run"]["tracking"] = "none"
    assert s2.run_tracking() == "none"


def test_production_screen_orders_stand_at_the_home_point(school):
    import tag_job as TJ

    class FakeS:
        map = PR.site_production(school)[0]
    tj = TJ.TagJob.__new__(TJ.TagJob)
    tj.s, tj.site = FakeS(), school
    prog = tj.production_program({"orders": {"Alpha": ["P11", "P01"], "Bravo": ["P22"]}, "dwell_s": 3, "home": True,
                                  "sync_spin": True})
    assert {"block": "wait", "robots": ["Alpha"], "seconds": 3.0, "label": "at point P11"} in prog["blocks"]
    assert not any(b.get("spot") in ("P11", "P22") for b in prog["blocks"])


def test_camera_plus_minus_counts_the_frame_error_corner_clicks_or_corner_tags():
    import types
    import run_plan as RP
    C = RP.Coordinator

    def cal(rp, k=None):
        fake = types.SimpleNamespace(cam_info=lambda: {"fit": {"rms_px": {"side": rp}, "sticker_sd_cm": {}}},
                                     CAM_CAL_DEFAULT_CM=C.CAM_CAL_DEFAULT_CM, PPC_DEFAULT=C.PPC_DEFAULT,
                                     FRAME_K=C.FRAME_K if k is None else k)
        return C.cam_cal(fake, "side", 4.0)
    old = cal({"tags": 2.0})                                          # no frame residual: as before (0.5 cm)
    assert old == pytest.approx(0.5)
    clicked = cal({"tags": 2.0, "corners": 2.7})                      # clicked corners 2.7 px: + 2 x 2.7 / 4 in quadrature
    assert clicked == pytest.approx(math.hypot(0.5, 2 * 2.7 / 4.0))
    assert cal({"tags": 2.0, "ref": 0.2}) == pytest.approx(math.hypot(0.5, 0.1))   # corner tags: small
    assert cal({"tags": 2.0, "corners": 2.7}, k=0) == pytest.approx(0.5)            # the switch: 0 = the old +-
