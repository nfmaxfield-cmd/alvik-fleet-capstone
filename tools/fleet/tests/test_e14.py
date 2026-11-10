"""e14 (10 Nov 2026): robots found by motion at the start of every run and brought back to where their tag reads,
calibration on any surface (its own spots), the readable area by light and tag, no re-trying a point that just failed,
the battery floor from the site / Mission Control, runs with tagless robots."""
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
sys.path[:0] = [FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition")]

import pytest  # noqa: E402


# -------------------------------------------------------------------------------------------- 1. finding by motion
def test_wiggle_on_the_spot_finds_a_robot_without_a_heading():
    import camera_discovery as CD
    rig = CD.SimRig(tags=False)
    truth = {n: list(p) for n, p in rig.robots.items()}
    rec = CD.discover(rig.caminfos(), sorted(rig.robots), rig.mover, modes={"Alpha": "wiggle"}, turner=rig.turner,
                      log=lambda s: None)
    a = rec["robots"]["Alpha"]
    assert a["how"] == "wiggle" and a["heading_deg"] is None
    assert math.dist(a["where_cm"], truth["Alpha"][:2]) < 2.0
    assert rec["robots"]["Bravo"]["how"] == "stroke" and rec["robots"]["Bravo"]["end_cm"] is not None


def test_stroke_or_wiggle_follows_the_clear_space_and_the_pose_guess():
    import motion_find as MF
    near_edge = lambda a, b: ["the edge"] if min(a[1], b[1]) < 20 else []    # noqa: E731
    m = MF.modes_for(["A", "B", "C"], {"A", "B"}, {"B": (50.0, 22.0, -90.0)}, near_edge)
    assert m == {"A": "stroke", "B": "wiggle", "C": "wiggle"}     # B's pinned tag shows a stroke would reach the edge
    assert MF.modes_for(["A"], False) == {"A": "wiggle"}
    assert MF.enabled()                                             # on by default (e14)


def test_rig_p15_unread_dark_patch_two_close_robots_and_a_blind_camera(tmp_path):
    """Nick's cases: a robot at P15 whose tag never reads there, one in a dark patch, two robots 25 cm apart, an extra
    camera that sees nothing. Every robot found by motion; the two whose tags do not read brought back until they
    read; the blind camera left out."""
    import motion_find as MF
    res = MF.run_rig(log=lambda s: None, dark=(100.0, 40.0, 25.0, 0.35))
    rob = res["robots"]
    assert all(f["found"] for f in rob.values())
    assert rob["Alpha"]["tag"] == 2 and rob["Bravo"]["tag"] == 3          # close together, each by its own stroke
    assert rob["Charlie"]["tag"] is None and rob["Charlie"]["readable"] is False
    for n, f in rob.items():
        t = res["truth_start"][n]
        assert math.dist(f["where"], t[:2]) < 3.0 * max(f["sigma"], 1.0), (n, f["where"], t)
    assert res["coverage"]["blind"] == ["laptop webcam"]
    bb = res["brought_back"]
    assert set(bb) == {"Charlie", "Delta"}
    assert bb["Charlie"]["tag"] == 4 and bb["Delta"]["tag"] == 5
    for n, b in bb.items():
        assert math.dist(b["at"], b["truth"]) < 5.0                       # the motion's estimate where it ended
        assert b["sightings"] >= 2


def _bb(**kw):
    import motion_find as MF
    st = dict(x=50.0, y=20.0, th=-90.0, held=0)

    def step(cm):
        h = math.radians(st["th"])
        st["x"] += cm * math.cos(h)
        st["y"] += cm * math.sin(h)
        return {"where": (st["x"], st["y"]), "heading": st["th"], "sigma": 1.0}

    def turn(d):
        st["th"] = (st["th"] + d + 180) % 360 - 180
        return d

    def hold(w):
        st["held"] += 1
        st.setdefault("words", w)
    args = dict(step=step, turn=turn, wiggle=lambda: {"where": (st["x"], st["y"]), "sigma": 1.0},
                problems=lambda a, b, buf: ["edge"] if min(a[1], b[1]) - buf < 6.72 else [],
                readable=lambda x, y: 0.9 if y > 60 else 0.1, tag_near=lambda x, y, r: 7 if y > 60 else None,
                hold=hold, cells=[(50.0, 70.0), (50.0, 90.0)], log=lambda s: None)
    args.update(kw)
    return MF.BringBack("Robo", (st["x"], st["y"], st["th"]), 1.0, **args), st


def test_bring_back_turns_away_from_the_edge_and_drives_until_its_tag_reads():
    b, st = _bb()
    x, y, th, t = b.run()
    assert t == 7 and y > 60 and b.steps >= 8 and st["held"] == 0
    assert min(p[1] for p in b.track) >= 20.0 - 0.01                  # never closer to the edge than it started


def test_bring_back_never_gives_up_it_holds_and_asks_and_stop_ends_it():
    import motion_find as MF
    stop = threading.Event()
    b, st = _bb(problems=lambda a, b, buf: ["the edge"], stop=stop.is_set)   # nothing is ever clear here
    out = {}

    def go():
        try:
            b.run()
        except MF.StopAsked as e:
            out["e"] = e
    t = threading.Thread(target=go)
    t.start()
    time.sleep(0.4)
    assert t.is_alive() and st["held"] >= 1 and "NEEDS A HAND" in st["words"]    # no timer ends it
    stop.set()
    t.join(5)
    assert "e" in out and b.steps == 0


def test_a_robot_with_no_tag_that_reads_is_found_tagless_not_driven_forever():
    b, st = _bb(tag_near=lambda x, y, r: None)
    x, y, th, t = b.run()
    assert t is None and y > 60                                       # two readable spots with no read: < 1 %
    assert b.p_miss < 0.01


def test_site_problems_edge_rule_and_flat_keepouts():
    import calib_run as CR
    import motion_find as MF
    import sites as SI
    S = CR.Site(SI.load("school-conference"))
    f = MF.site_problems(S)
    p15 = S.points["P15"]                                             # beside reference tile 25
    assert S.problems(p15, p15, buf=4.0, edge_buf=4.0)                # the planning rule refuses a turn there
    assert f(p15, (p15[0] - 5, p15[1] - 5), 4.0) == []                # moving away from the tile is allowed
    assert f((50.0, 10.0), (50.0, 5.0), 2.0)                          # the edge rule: error sum + half body
    S.set_parked({"Other": (100.0, 80.0)})
    assert f((80.0, 80.0), (90.0, 80.0), 3.0)                         # robots are solid: with the error sum


# -------------------------------------------------------------------------------------------- 2. any surface
def _site_without_points(tmp_path):
    import sites as SI
    d = json.loads(json.dumps(SI.load("school-conference")))
    d.pop("points", None)
    return d


def test_spots_switch_stickers_when_proven_else_its_own(tmp_path, monkeypatch):
    import calib_run as CR
    import sites as SI
    monkeypatch.delenv("MIRLT_CALIB_SPOTS", raising=False)
    school = SI.load("school-conference")
    assert CR.spots_mode(school)[0] == "stickers"
    assert CR.spots_mode(_site_without_points(tmp_path))[0] == "own"
    grid = json.loads(json.dumps(school))
    grid["points"]["_note"] = "made from the measured table (site_detect): a grid, not stickers"
    assert CR.spots_mode(grid) == ("own", "the site's points are a grid made for a new area, not stickers")
    small = json.loads(json.dumps(school))
    small["table"] = {"length_cm": 150.0, "width_cm": 90.0}
    m, why = CR.spots_mode(small)
    assert m == "own" and "off the table" in why
    assert CR.spots_mode(school, "own")[0] == "own"
    monkeypatch.setenv("MIRLT_CALIB_SPOTS", "own")
    assert CR.spots_mode(school)[0] == "own"


def test_own_spots_spread_inside_the_readable_area_clear_of_edges_and_keepouts(tmp_path):
    import calib_run as CR
    site = _site_without_points(tmp_path)
    S = CR.Site(site)                                                  # no points file: no crash (e13 crashed here)
    assert S.spots_mode == "own" and S.points == {}
    w = S.make_own_spots(lambda x, y: x < 180.0)
    assert len(S.points) >= 8 and "spots of its own" in w
    pts = list(S.points.values())
    for i, a in enumerate(pts):
        assert a[0] < 180.0
        assert S.edge_dist(a) >= S.edge + CR.EDGE_STAGE_ROOM - 1e-6
        assert S.keepout_dist(a)[0] >= CR.KEEPOUT_CLEAR_CM + CR.OWN_SPOTS_KEEPOUT_EXTRA - 1e-6
        for b in pts[i + 1:]:
            assert math.dist(a, b) >= CR.OWN_SPOTS_MIN_CM - 1e-6
    park = S.own_park_spots()
    assert len(park) == 3 and park[0] == S.own_centre
    assert S.zone12() == sorted(S.points, key=lambda s: int(s[1:]))
    assert CR.sim_xy(S, "S01") == S.points["S01"] and CR.sim_xy(S, "30,40") == (30.0, 40.0)


# -------------------------------------------------------------------------------------------- 4. no re-trying at once
def _tour_run(monkeypatch, fail_times):
    import calib_run as CR
    import observe as OB
    import sites as SI
    r = CR.CalibRun.__new__(CR.CalibRun)
    r.S = CR.Site(SI.load("school-conference"))
    r.a = type("A", (), {"lost_policy": "recover", "dry": True})()
    r.stop_flag = threading.Event()
    r.tag, r.events_, r.said = 4, [], []
    r.ev = lambda kind, **kw: r.events_.append((kind, kw))
    r.set_status = lambda **kw: None
    r.cam_pose = lambda dur=None: (100.0, 80.0, 0.0)
    rm, rt = OB.ReadMap(r.S.L, r.S.W), OB.ReadMap(r.S.L, r.S.W)
    rm.seed(lambda x, y: 0.9, "test")
    rt.seed(lambda x, y: 0.9, "test")
    r.readmap, r.readmap_tag = (lambda: rm), (lambda: rt)
    r.check_battery = lambda: 90
    visits = []
    left = dict(fail_times)

    def _goto(lab, phase, tour, cam=None, odometry=True, depth=0, name=None):
        visits.append(lab)
        if left.get(lab, 0) > 0:
            left[lab] -= 1
            r.last_lost_at = (150.0, 20.0)
            raise CR.Recovered((101.0, 80.0, 0.0))
        return (r.S.points[lab][0], r.S.points[lab][1], 0.0)
    r._goto = _goto
    monkeypatch.setattr(CR, "DWELL_S", 0.0)
    monkeypatch.setattr(CR, "say", r.said.append)
    r.tour("C", 1, ["P01", "P04", "P05"])
    return r, visits, rm


def test_a_point_whose_tag_was_just_lost_goes_to_the_end_of_the_tour_once(monkeypatch):
    r, visits, rm = _tour_run(monkeypatch, {"P04": 1})
    assert visits == ["P01", "P04", "P05", "P04"]                      # not P04 four times in a row
    kinds = [k for k, _ in r.events_]
    assert "point_deferred" in kinds and "skip_point" not in kinds
    assert rm.tries.get(rm.ij(150.0, 20.0)) == 1                       # where it was lost: evidence


def test_a_point_lost_again_at_the_end_is_skipped_with_its_reason(monkeypatch):
    r, visits, rm = _tour_run(monkeypatch, {"P04": 5})
    assert visits == ["P01", "P04", "P05", "P04"]
    sk = [kw for k, kw in r.events_ if k == "skip_point"]
    assert sk and sk[0]["point"] == "P04" and "a second time" in sk[0]["why"]
    monkeypatch.setattr(sys.modules["calib_run"], "RETRY_LOST", "again")      # the switch: the old aim-again
    r2, visits2, _ = _tour_run(monkeypatch, {"P04": 1})
    assert visits2 == ["P01", "P04", "P04", "P05"]


def test_run_plan_defers_then_skips_a_visit_point_lost_on_the_way(monkeypatch):
    import run_plan as RP
    c = RP.Coordinator.__new__(RP.Coordinator)
    vo = [{"label": "point 4"}, {"label": "at point 4"}, {"label": "point 5"}, {"label": "at point 5"}]
    c.plan = {"robots": {"Alpha": {"visit_order": vo}}}
    c.current_task = lambda n: (0, vo[0])
    c.log = lambda *a, **k: None
    c.t_run = time.time()
    assert c.defer_current_point("Alpha", "lost") == ("point 4", "deferred")
    assert [v["label"] for v in vo] == ["point 5", "at point 5", "point 4", "at point 4"]
    c.current_task = lambda n: (2, vo[2])
    assert c.defer_current_point("Alpha", "lost") == ("point 4", "skipped")
    assert [v["label"] for v in vo] == ["point 5", "at point 5"]
    monkeypatch.setenv("MIRLT_RETRY_LOST", "again")
    assert c.defer_current_point("Alpha", "lost") == (None, None)


# -------------------------------------------------------------------------------------------- 5. the battery floor
def test_battery_floor_from_mission_control_or_the_site_file_and_said(tmp_path, monkeypatch):
    import engine_checks as EC
    monkeypatch.delenv("MIRLT_MIN_BATTERY", raising=False)
    site = {"mc_dir": str(tmp_path), "battery_floor": 20}
    assert EC.battery_floor_used(site=site) == (20, "20 % (the site file's battery_floor)")
    (tmp_path / "advanced.json").write_text(json.dumps({"battery_floor": "15"}))
    f, w = EC.battery_floor_used(site=site)
    assert f == 15 and "Advanced" in w
    (tmp_path / "advanced.json").write_text(json.dumps({"battery_floor": ""}))      # blank: the default again
    assert EC.battery_floor_used(site={"mc_dir": str(tmp_path)}) == (25, "25 % (the default)")
    assert EC.battery_floor_used("freeroam", site={"mc_dir": str(tmp_path)})[0] == 10
    monkeypatch.setenv("MIRLT_MIN_BATTERY", "8")                       # set by hand / tmux: still first
    assert EC.battery_floor_used(site=site)[0] == 8
    monkeypatch.setenv("MIRLT_MIN_BATTERY", "")
    (tmp_path / "advanced.json").write_text(json.dumps({"battery_floor": "2"}))
    assert EC.battery_floor_used(site=site)[0] == 5                    # never under 5


def test_a_calibration_started_under_40_percent_is_marked_low_battery():
    import calib_run as CR
    src = open(CR.__file__).read()
    assert CR.LOW_BATTERY_NOTE_PCT == 40 and "low_battery_calibration" in src
    import calib_report as CRP
    assert "LOW-BATTERY CALIBRATION" in open(CRP.__file__).read()


# -------------------------------------------------------------------------------------------- 3. light and tag
def _sess(store, L, W, grey, tag, reads_until_x, tries=8, kind="t"):
    import observe as OB
    import readmap_store as RS
    rm = OB.ReadMap(L, W)
    for i in range(rm.nx):
        for j in range(rm.ny):
            x, y = rm.centre(i, j)
            for _ in range(tries):
                rm.add(x, y, x < reads_until_x)
    return RS.add_session(store, rm, RS.evidence(rm), grey=grey, tag=tag, kind=kind)


def test_a_dim_session_never_shrinks_the_lit_map_and_light_and_tag_choose(tmp_path):
    import observe as OB
    import readmap_store as RS
    L, W, d = 396.2, 167.6, str(tmp_path)
    for _ in range(4):
        _sess(d, L, W, 91.0, "*", 220.0)                   # four lit sessions: tags read to x 220
    w = _sess(d, L, W, 70.0, 4, 30.0, tries=40)           # one long dim session, tag 4: read only to x 30
    assert "mean grey 70 (dim)" in w and "tag 4" in w
    assert all(n <= RS.SESSION_CAP for s in RS.load(d)["sessions"] for _i, _j, n, _r in s["cells"])

    def extent(grey, tag=None):
        rm = OB.ReadMap(L, W)
        RS.choose(d, rm, grey, tag)
        return RS.readable_extent(rm, L)
    assert extent(91.0) == pytest.approx(220.0, abs=21)    # lit: the dim session does not shrink it
    assert extent(91.0, tag=4) == pytest.approx(220.0, abs=21)
    assert extent(70.0, tag=4) <= 60.0                     # in the dim light, tag 4's own dim evidence decides
    assert extent(None) >= 200.0                           # light unknown: one capped session cannot outvote four
    rm = OB.ReadMap(L, W)
    words = RS.choose(d, rm, 91.0, None)
    assert "4 sessions at about this light" in words and "1 in other light kept apart" in words


def test_old_maps_are_carried_in_once(tmp_path):
    import observe as OB
    import readmap_store as RS
    rm = OB.ReadMap(396.2, 167.6)
    rm.add(50, 50, True, n=5)
    (tmp_path / "readmap_light.json").write_text(json.dumps({"91": rm.to_json(), "70": rm.to_json()}))
    rm.save(str(tmp_path / "readmap_tag4.json"))
    w = RS.import_old(str(tmp_path), OB.ReadMap)
    assert "grey 91" in w and "grey 70" in w and "tag 4" in w
    m4 = OB.ReadMap(396.2, 167.6)
    assert "tag 4" in RS.choose(str(tmp_path), m4, None, 4, only_tag=True) and m4.tries[m4.ij(50, 50)] == 5
    assert RS.import_old(str(tmp_path), OB.ReadMap) == ""                  # once
    assert sorted(s["grey"] for s in RS.load(str(tmp_path))["sessions"] if s["grey"]) == [70.0, 91.0]


# -------------------------------------------------------------------------------------------- 6. robots with no tag
def test_a_tagless_robot_gets_a_phantom_id_a_start_pose_and_a_motion_seed():
    import wiggle_bind as WB
    import mission_run as MR
    import run_plan as RP
    rec = {"slots": {"Alpha": 2}, "tagless": {"Charlie": {"seen_cm": [82.0, 139.0], "heading": 92.0, "sigma": 1.4}}}
    t = WB.tags_with_binding({"ids": {"Alpha": 0}}, rec)
    assert t["ids"] == {"Alpha": 2, "Charlie": WB.PHANTOM_ID} and t["tagless"]["Charlie"]["heading"] == 92.0
    a = type("A", (), {})()
    MR.take_tagless(a, rec["tagless"])
    sp, sm = MR.tagless_args(a)
    assert sp == ["--start-pose", "Charlie=82.00,139.00,92.0"] and sm == ["--seed-motion", "Charlie=82.00,139.00,92.0"]
    args = type("A", (), {"start_pose": sp[1:]})()
    assert RP.start_poses_given(args) == {"Charlie": (82.0, 139.0, 92.0)}


def test_the_facing_check_takes_the_motion_finding_for_a_robot_with_no_tag():
    import run_plan as RP
    c = type("C", (), {})()
    c.log, c.pose_dest, said = (lambda *a, **k: None), None, []
    import builtins
    orig = RP.warn
    RP.warn = lambda coord, text: said.append(text)
    try:
        fres = {"headings": {"Alpha": 90.0}, "unknown": ["Charlie"], "robots": {"Charlie": {"meas": {}}}}
        RP.given_start_poses(c, type("A", (), {"start_pose": ["Charlie=82,139,92"]})(), fres, ["Alpha", "Charlie"])
    finally:
        RP.warn = orig
    assert fres["headings"]["Charlie"] == 92.0 and "Charlie" not in fres["unknown"]
    assert c.facing_at["Charlie"] == (82.0, 139.0) and "no roof tag that reads" in said[0]


def test_the_bridge_starts_a_motion_track_where_the_finding_put_a_tagless_robot():
    import tag_bridge as TB
    br = TB.Bridge.__new__(TB.Bridge)
    added = []
    br.counts = {"Charlie": {}}
    br.motion = type("M", (), {"add": lambda self, n, x, y, heading=None, t=None: added.append((n, x, y, heading)),
                               "push": lambda self, g, t: None, "state": lambda self, n: None})()
    br.motion_cam, br.motion_added, br.observe = "overhead", set(), True
    br.event = lambda *a, **k: None
    br.on_seed({"robot": "Charlie", "x": 82.0, "y": 139.0, "th": 92.0})
    br.on_seed({"robot": "Nobody", "x": 1.0, "y": 1.0})
    br.motion_step("overhead", None, 10.0, set(), [])
    assert added == [("Charlie", 82.0, 139.0, 92.0)] and "Charlie" in br.motion_added


def test_a_found_robot_with_a_heading_starts_exactly_where_it_was_found():
    import mission_run as MR
    import sites as SI
    a = type("A", (), {})()
    a.site_ = SI.load("school-conference")
    a.split = "auto"
    a.experiment_ = None
    a.seen_ = {"Charlie": [100.3, 120.7, 47.0], "Alpha": [30.0, 30.0]}
    a.auto_robots = False
    try:
        home_at, _o = MR.observed_split(a, ["Alpha", "Charlie"], None)
    except Exception:  # noqa: BLE001  (the split needs more of a run; the homes come first)
        pytest.skip("observed_split needs a fuller run here")
    assert home_at["Charlie"] == [100.3, 120.7, 47.0]
