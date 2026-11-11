"""Build e15 (11 Nov 2026, school 10 Nov 18:54): each roof tag's turn on its robot remembered across runs, the facing
checks on the corrected heading, the patient heading check, the start crawl that measures an unverified turn.
Simulator stand-ins only; no robot, no Pi."""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment"),
          os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import tag_heading_store as THS     # noqa: E402


def w180(a):
    return (a + 180.0) % 360.0 - 180.0


# ------------------------------------------------------------------------------------------------- the store
def test_the_store_remembers_confirms_and_lets_new_evidence_win(tmp_path):
    p = str(tmp_path / "tag_headings.json")
    assert THS.get(p, 2) is None
    r = THS.record(p, 2, -180.0, "the binding's heading check", robot="AL-1DC0")
    assert r["result"] == "new" and THS.get(p, 2)["offset_deg"] == -180.0
    r = THS.record(p, 2, 179.0, "run_plan's heading watch (re-learned)")       # the same turn (wrapped)
    assert r["result"] == "agrees" and THS.get(p, 2)["n"] == 2
    r = THS.record(p, 2, 0.0, "the binding's heading check")                    # re-mounted straight
    e = THS.get(p, 2)
    assert r["result"] == "changed" and e["offset_deg"] == 0.0 and e["n"] == 1
    assert e["changes"][-1]["from"] == -180.0 and "evidence wins" in r["words"]
    assert THS.get(p, 3) is None and THS.record("", 3, 90, "x")["result"] == "new"   # no file: nothing kept
    assert THS.get("", 3) is None


def test_the_switch_and_the_site_setting(monkeypatch):
    monkeypatch.delenv("MIRLT_TAG_HEADING_MEMORY", raising=False)
    assert THS.enabled({}) and not THS.enabled({"tags": {"heading_memory": False}})
    monkeypatch.setenv("MIRLT_TAG_HEADING_MEMORY", "0")
    assert not THS.enabled({})
    assert THS.path_for({"conf_dir": "/x"}) == "/x/tag_headings.json" and THS.path_for({}, "") == ""


def test_free_roams_proof_drive_is_harvested_on_top_of_the_bridges_turn(tmp_path):
    p = str(tmp_path / "s.json")
    d = tmp_path / "discovery.json"
    d.write_text(json.dumps({"robots": {"Alpha": {"tag_off": 178.6}, "Bravo": {"tag_off": 1.2}}}))
    tags = {"ids": {"Alpha": 2, "Bravo": 4}, "yaw_off": {"4": -180.0}}
    THS.harvest_discovery(p, str(d), tags)
    assert abs(THS.get(p, 2)["offset_deg"]) == 180.0 and THS.get(p, 4)["offset_deg"] == -180.0


# ------------------------------------------------------------------------------------------------- the facing check
def _coord(**bots):
    return types.SimpleNamespace(bots={n: types.SimpleNamespace(**kw) for n, kw in bots.items()})


def test_facing_uses_the_turn_this_run_knows_never_the_raw_tag(monkeypatch):
    import facing as FC
    monkeypatch.delenv("MIRLT_FACING_RAW", raising=False)
    tags = {"ids": {"Alpha": 2, "Bravo": 4}, "yaw_off": {}, "yaw_state": {"2": "unknown", "4": "remembered"}}
    t, trust = FC.corrected_tags(_coord(Alpha=dict(yaw_fix=-180.0, yaw_known=True), Bravo={}), tags, ["Alpha", "Bravo"])
    assert t["yaw_off"]["2"] == -180.0 and trust == {"Alpha": "measured", "Bravo": "remembered"}
    monkeypatch.setenv("MIRLT_FACING_RAW", "1")        # the old way: the file alone
    t, trust = FC.corrected_tags(_coord(Alpha=dict(yaw_fix=-180.0, yaw_known=True)), tags, ["Alpha"])
    assert "2" not in t["yaw_off"] and trust["Alpha"] == "unknown"


def test_unverified_turns_are_crawled_remembered_ones_only_before_a_big_turn():
    import facing as FC
    assert FC.unverified_words("Alpha", "unknown", 2.0)
    assert FC.unverified_words("Alpha", "remembered", 171.0) and FC.unverified_words("Alpha", "motion", 90.0)
    assert FC.unverified_words("Alpha", "remembered", 9.0) is None
    assert FC.unverified_words("Alpha", "measured", 171.0) is None and FC.unverified_words("A", "typed", 171) is None


def test_a_recheck_after_a_relearn_reads_the_turned_tag_the_right_way():
    """The school's facing-recheck-N after HEADING RE-LEARNED: the tag (turned 180 on the robot) drawn in the simulator's
    picture; the robot faces +90 (its start heading). With the run's yaw_fix the check says 'faces 90', not
    'backwards'."""
    import facing as FC
    import tag_cam as CAMS
    cam = CAMS.TagCamera((100, 80, 190), (100.01, 80, 0), size=(1280, 960), hfov_deg=70, seed=6, table=(0, 0, 300, 200))
    # the tag reads 180 deg less than the nose: drawn at -90 for a nose at +90
    img = cam.render([dict(x=90, y=70, th=math.radians(-90.0), z=6.0, family="tag25h9", id=2, tag_cm=8.0,
                           roof_cm=9.5)])

    class Grab:
        def between(self, c, t0, t1):
            return [img] * 3
    FC.TAG_LOOK_S = 0.0
    base = {"family": "tag25h9", "tag_cm": 8.0, "roof_cm": 9.5, "height_cm": 6.0, "ids": {"Alpha": 2},
            "yaw_state": {"2": "unknown"}}
    raw = FC.tag_check(["Alpha"], Grab(), {"overhead": cam.true_model()}, base, assumed={"Alpha": 90.0},
                       say=lambda s: None)
    assert not raw["robots"]["Alpha"]["verdict"]["ok"]                 # raw: backwards
    tags, trust = FC.corrected_tags(_coord(Alpha=dict(yaw_fix=-180.0, yaw_known=True)), base, ["Alpha"])
    res = FC.tag_check(["Alpha"], Grab(), {"overhead": cam.true_model()}, tags, assumed={"Alpha": 90.0},
                       say=lambda s: None)
    m = res["robots"]["Alpha"]["meas"]
    assert res["robots"]["Alpha"]["verdict"]["ok"] and abs(w180(m["heading"] - 90.0)) < 3 and m["tag_off"] == -180.0
    assert trust["Alpha"] == "measured"


# ------------------------------------------------------------------------------------------------- the heading check
class World:
    """A robot whose nose points at `nose` deg with its roof tag turned `turn` deg (the tag reads nose - turn... the
    offset to ADD to the tag is `turn`); reads lost where `lost(x, y, k)` says (k: the measure call's number)."""

    def __init__(self, nose=90.0, turn=-180.0, lost=lambda x, y, k: False):
        self.x, self.y, self.nose, self.turn, self.lost, self.k = 100.0, 80.0, nose, turn, lost, 0
        self.legs = []
        self.yaw_off = {}
        self.hist = {2: [(i * 0.1, 0, 0, 0) for i in range(10)]}
        import threading
        self.lock = threading.Lock()

    # the robot
    name = "AL-1DC0"

    def leg(self, kind, value, on_tick=None):
        self.legs.append((kind, value))
        if kind == "turn":
            self.nose = w180(self.nose + value)
        else:
            self.x += value * math.cos(math.radians(self.nose))
            self.y += value * math.sin(math.radians(self.nose))
        return {"result": "done", "info": ""}

    # the eye
    def measure(self, tag, dur=1.2):
        self.k += 1
        if self.lost(self.x, self.y, self.k):
            return None
        return (self.x, self.y, w180(self.nose - self.turn + self.yaw_off.get(tag, 0.0)), 10)


def _patient(world, monkeypatch, **kw):
    import calib_run as CR
    monkeypatch.setattr(CR.time, "sleep", lambda s: None)
    pose = world.measure(2)[:3]
    world.hist = {2: [(__import__("time").time() - 1 + i * 0.1, 0, 0, 0) for i in range(10)]}
    return CR.heading_check_patient(world, world, 2, pose, lambda a, b: [], log=lambda s: None, **kw)


def test_a_heading_check_whose_tag_is_lost_for_a_moment_reads_it_again_and_measures(monkeypatch):
    w = World(lost=lambda x, y, k: k == 2)              # lost just after the nudge (the school's 18:54)
    r = _patient(w, monkeypatch)
    assert r["state"] == "measured" and r["offset"] == -180.0 and "read again" in r["how"]


def test_a_spot_where_the_tag_does_not_read_backs_out_and_nudges_the_other_way(monkeypatch):
    w = World(lost=lambda x, y, k: y > 83.0)            # ahead of it the tag is not read (the nose points +y)
    r = _patient(w, monkeypatch)
    assert r["state"] == "measured" and r["offset"] == -180.0
    assert ("go", -6.0) in w.legs and w.y < 80.0         # backed out, then nudged backwards where it reads


def test_lost_for_good_the_remembered_turn_is_the_starting_guess_and_the_robot_is_back(monkeypatch):
    w = World(lost=lambda x, y, k: k >= 2)
    r = _patient(w, monkeypatch, remembered={"offset_deg": -180.0, "n": 3, "last_seen": "2026-11-10T19:04"})
    assert r["state"] == "remembered" and r["offset"] == -180.0 and w.yaw_off[2] == -180.0
    assert abs(w.y - 80.0) < 0.01                        # it backed the nudge out: where it stood


def test_lost_for_good_with_nothing_remembered_is_unknown_never_the_raw_heading(monkeypatch):
    w = World(lost=lambda x, y, k: k >= 2)
    r = _patient(w, monkeypatch)
    assert r["state"] == "unknown" and r["offset"] is None and 2 not in w.yaw_off


def test_a_measured_turn_overrides_a_wrong_memory(monkeypatch):
    w = World(turn=0.0)                                   # re-mounted straight; the memory says -180
    r = _patient(w, monkeypatch, remembered={"offset_deg": -180.0, "n": 3})
    assert r["state"] == "measured" and r["offset"] == 0.0


# ------------------------------------------------------------------------------------------------- run_plan
def _rp_coord(tmp_path, yaw_off=None, store=True):
    tags = {"ids": {"Alpha": 2}, "yaw_off": yaw_off or {}}
    tp = tmp_path / "tags.json"
    tp.write_text(json.dumps(tags))
    args = types.SimpleNamespace(facing_tags=str(tp), tag_heading_store=str(tmp_path / "th.json") if store else "",
                                 fix_hz=5.0)
    logs = []
    c = types.SimpleNamespace(args=args, bots={"Alpha": types.SimpleNamespace(name="Alpha", yaw_fix=0.0, pose=None)},
                              log=lambda e, **kw: logs.append((e, kw)))
    return c, logs


def test_the_start_crawl_measures_the_turn_and_remembers_it(tmp_path):
    import run_plan as RP
    c, logs = _rp_coord(tmp_path)
    fres = {"robots": {"Alpha": {"meas": {"heading": -99.0, "tag_off": 0.0}}}}
    tot = RP.learn_turn_from_crawl(c, "Alpha", fres, 81.5)
    b = c.bots["Alpha"]
    assert tot == -180.0 and b.yaw_fix == -180.0 and b.yaw_known
    assert THS.get(c.args.tag_heading_store, 2)["offset_deg"] == -180.0


def test_a_remembered_turn_the_crawl_disproves_is_replaced(tmp_path):
    """Re-mounted: the tags file carries the remembered -180, the tag now reads with the nose (0)."""
    import run_plan as RP
    c, logs = _rp_coord(tmp_path, yaw_off={"2": -180.0})
    THS.record(c.args.tag_heading_store, 2, -180.0, "earlier run")
    fres = {"robots": {"Alpha": {"meas": {"heading": -88.0, "tag_off": -180.0}}}}   # tag raw 92, corrected -88
    RP.learn_turn_from_crawl(c, "Alpha", fres, 91.0)
    assert abs(c.bots["Alpha"].yaw_fix) == 180.0            # on top of the file's -180: 0 in all
    e = THS.get(c.args.tag_heading_store, 2)
    assert e["offset_deg"] == 0.0 and e["changes"][-1]["from"] == -180.0


def test_run_plans_relearn_is_remembered(tmp_path):
    import run_plan as RP
    c, logs = _rp_coord(tmp_path)
    sent = []
    c.t_run = 1.0
    c.send = lambda n, m: sent.append(m)
    c.say = lambda m: None
    b = c.bots["Alpha"]
    b.pose = (100.0, 80.0, 90.0)
    # the robot drives +y (its own pose agrees), the camera's raw tag says it faces -90 (turned 180)
    y = 80.0
    for k in range(40):
        y += 1.0
        b.pose = (100.0, y, 90.0)
        RP.Coordinator.observe_fix(c, b, {"x": 100.0, "y": y, "th": -90.0})
        if getattr(b, "yaw_known", False):
            break
    assert b.yaw_known and abs(b.yaw_fix) == 180.0
    assert abs(THS.get(c.args.tag_heading_store, 2)["offset_deg"]) == 180.0


# ------------------------------------------------------------------------------------------------- mission_run
def test_a_run_starts_from_the_remembered_turn_else_unknown(tmp_path, monkeypatch):
    import mission_run as MR
    monkeypatch.delenv("MIRLT_TAG_HEADING_MEMORY", raising=False)
    monkeypatch.setattr(MR, "say", lambda s: None)
    st = tmp_path / "store"
    st.mkdir()
    THS.record(str(st / THS.FILE), 2, -180.0, "earlier run")
    args = types.SimpleNamespace(site_={}, tag_heading_store=str(st), dry=True, robots="Alpha,Bravo",
                                 tags_={"ids": {"Alpha": 2, "Bravo": 4}})
    MR.apply_tag_memory(args, str(tmp_path))
    t = args.tags_
    assert t["yaw_off"]["2"] == -180.0 and t["yaw_state"] == {"2": "remembered", "4": "unknown"}
    # a binding that measured this run is kept as measured
    args.tags_ = {"ids": {"Alpha": 2}, "yaw_off": {"2": 0.0}, "yaw_state": {"2": "measured"}}
    MR.apply_tag_memory(args, str(tmp_path))
    assert args.tags_["yaw_state"]["2"] == "measured" and args.tags_["yaw_off"]["2"] == 0.0
    # a dry run with no store named keeps its own (nothing carried over)
    args2 = types.SimpleNamespace(site_={}, tag_heading_store="", dry=True)
    assert MR.heading_store_path(args2, str(tmp_path / "s")) == str(tmp_path / "s" / THS.FILE)
    monkeypatch.setenv("MIRLT_TAG_HEADING_MEMORY", "0")
    assert MR.heading_store_path(args, str(tmp_path)) == ""


def test_the_bindings_turns_reach_the_tags_block_with_how_they_are_known():
    import wiggle_bind as WB
    rec = {"slots": {"Alpha": 2, "Bravo": 4}, "yaw_off": {"Alpha": -180.0},
           "yaw_state": {"Alpha": "remembered", "Bravo": "unknown"}, "yaw_how": {"Alpha": "remembered (x)"}}
    t = WB.tags_with_binding({"ids": {}}, rec)
    assert t["yaw_off"] == {"2": -180.0} and t["yaw_state"] == {"2": "remembered", "4": "unknown"}
