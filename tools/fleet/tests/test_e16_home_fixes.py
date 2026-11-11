"""Build e16, items 9a and 9b (home free roam 20261111-082651-freeroam, Mission Control's screen at the end).
No robot, no Pi."""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "pi")):
    if p not in sys.path:
        sys.path.insert(0, p)

import start_job as SJ     # noqa: E402


def _starter(site):
    st = SJ.Starter.__new__(SJ.Starter)
    st.S = types.SimpleNamespace(tag=None, site=site, frame=lambda cam: None)
    st.practice_forced = False
    return st


def test_9a_the_click_uses_the_floor_calibration_where_there_is_no_tag_model(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "home")            # home: roam1 (the repo's floor calibration) as the fallback
    monkeypatch.setenv("MIRLT_PLACE_DIR", str(tmp_path / "place"))
    st = _starter({"name": "home"})
    m = st.place_model("overhead")
    assert m is not None and st.place_kinds["overhead"] == "floorcal"
    import e02_camera as CAM
    assert st.place_z("overhead") == CAM.ROBOT_TOP_CM
    # a click at the picture spot of (60, 60) at the robot's top lands there
    u, v = m.project((60.0, 60.0, CAM.ROBOT_TOP_CM))
    x, y = m.to_floor(float(u), float(v), CAM.ROBOT_TOP_CM)
    assert abs(x - 60.0) < 0.1 and abs(y - 60.0) < 0.1
    s = st.place_state()
    assert "overhead" in s["cams"]                       # the card shows the picture, not "No camera model here"


def test_9a_no_model_at_all_still_says_so(tmp_path, monkeypatch):
    monkeypatch.setenv("MIRLT_CONF_DIR", str(tmp_path))
    monkeypatch.setenv("MIRLT_SITE", "school-conference")   # another site, no floor calibration saved: none
    st = _starter({"name": "school-conference"})
    assert st.place_model("overhead") is None


def test_9b_a_standing_robots_doubt_does_not_grow_with_its_fixs_age():
    import freeroam_core as FC
    c = FC.Controller(["Center"], (0.0, 0.0, 122.24, 121.92), lambda n, s: None, lambda *a: None,
                      pose_source="blend")
    r = c.R["Center"]
    for k in range(10):
        c.t = 1.0 + 0.1 * k
        c.on_follow("Center", c.t, "idle", 61.0, 100.0, 90.0)
        c.on_fix("Center", c.t, 61.0, 100.0, None)
    r.mode = "idle"
    for k in range(30):                                   # 3 s, no picture (lights tracking: a still robot), standing
        c.t = 2.0 + 0.1 * k
        c.on_follow("Center", c.t, "idle", 61.0, 100.0, 90.0)
    assert c.standing(r) and c.unc(r) < 2.0                # was 1 + 10 cm/s x 3 s = 31 cm
    j = FC.SR.edge_judgement(FC.edge_dist(c.rect, 61.0, 100.0) - c.turn_need(r), c.fresh(r), 0.0)
    assert j["state"] == "clear"                           # 22 cm from the y = 122 edge is clear
    old = FC.STAND_UNC
    try:
        FC.STAND_UNC = False
        assert c.unc(r) > 20.0                             # the switch: as before
    finally:
        FC.STAND_UNC = old
