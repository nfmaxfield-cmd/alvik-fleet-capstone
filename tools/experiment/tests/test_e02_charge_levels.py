"""The runner's battery learner and dock levels (Nick, 2026-10-25 13:03: the go / leave levels adapt as battery data
comes in): battery.csv marks robots sent to sleep as asleep, every tick refits the drains and puts each robot's levels
into e02_state.json "charge_levels" and the session log, and the live view's robot cards show them. No robots: fake
runners with the attributes these methods read."""
import os
import threading
import time
import types

import sys

import e02_testkit as KIT  # noqa: F401  (sets the import paths)
import e02 as E
import e02_plan as PL
import e02_view as V

sys.path.insert(0, os.path.join(E.TOOLS, "dock"))
import battery_rates as BR  # noqa: E402


def _battery_csv(folder, t0, rows=40, step=120):
    lines = ["t,robot,role,state,still_s,battery,phase,source"]
    for i in range(rows):
        t = t0 + step * i
        lines.append("%.1f,Left,driving,%s,%d,%d,route,status" % (t, "moving" if i % 5 == 0 else "still",
                                                                   0 if i % 5 == 0 else step, round(95 - 16 * i * step / 3600)))
        lines.append("%.1f,Center,driving,still,%d,%d,route,status" % (t + 0.5, step, round(90 - 14 * i * step / 3600)))
        lines.append("%.1f,Right,idle,asleep,0,%d,route,status" % (t + 1, round(80 - 3 * i * step / 3600)))
    with open(os.path.join(folder, "battery.csv"), "w") as f:
        f.write("\n".join(lines) + "\n")


def _fake(folder, said):
    return types.SimpleNamespace(a=types.SimpleNamespace(dry_run=True, battery_watch_s=1), dir=folder,
                                 st=PL.State(folder), unavailable=set(), asleep_names={"Right"},
                                 say=lambda m, quiet=False: said.append((m, quiet)))


def test_levels_tick_fills_the_state_and_the_log(tmp_path, monkeypatch):
    d = tmp_path / "e02-20261025-140000-dryrun"
    d.mkdir()
    _battery_csv(str(d), BR.session_start(str(d)))
    monkeypatch.setattr(E, "REPO", str(tmp_path))          # the history goes to tmp/logs, not the repo's logs
    said = []
    r = _fake(str(d), said)
    out = E.Runner.charge_levels_tick(r, ["Left", "Center"])
    assert set(out) == {"Left", "Center", "Right"}
    assert 0 < out["Left"]["go"] < out["Left"]["leave"] and out["Left"]["still_from"] == "this session"
    assert out["Right"]["asleep_from"] == "this session"
    st = PL.State.load(str(d))
    assert st.get("charge_levels")["Center"]["go"] == out["Center"]["go"]         # saved in e02_state.json
    assert any(m.startswith("  battery: Left: go to the dock at") and q for m, q in said)
    assert os.path.exists(tmp_path / "logs" / "battery_learned-dryrun.json")
    said.clear()
    E.Runner.charge_levels_tick(r, ["Left", "Center"])
    assert not any("go to the dock" in m for m, _ in said)                      # unchanged levels: not repeated


def test_battery_csv_marks_sleeping_robots_asleep_counts_seconds_moved_and_ticks_the_levels(tmp_path):
    d = tmp_path / "e02-20261025-150000-dryrun"
    d.mkdir()
    said, ticks = [], []
    now = time.time()
    r = _fake(str(d), said)
    r.link = types.SimpleNamespace(robots=lambda: ["Left", "Right"], send_name=lambda n, c: None)
    r.listener = types.SimpleNamespace(battery={"Left": (now, 80), "Right": (now, 70)})
    r.leg_live, r.moves, r.hold, r.phase = {"Left": now}, {}, threading.Event(), "route"     # Left is driving a leg
    r.charge_levels_tick = lambda names: ticks.append(list(names))
    path = d / "battery.csv"

    def rows():
        return [ln.split(",") for ln in open(path).read().splitlines()[1:]] if os.path.exists(path) else []

    E.Runner.start_battery_watch(r, ["Left"])
    try:
        for _ in range(60):
            if len(rows()) >= 2:
                break
            time.sleep(0.1)
        time.sleep(4.5)                                 # two more ticks while Left drives
        r.listener.battery["Left"] = (time.time(), 79)
        for _ in range(60):
            if len(rows()) >= 3:
                break
            time.sleep(0.1)
    finally:
        r._bat_stop = True
    head = open(path).readline().strip().split(",")
    assert head[-1] == "moved_s"
    got = rows()
    assert {q[1]: q[3] for q in got[:2]} == {"Left": "moving", "Right": "asleep"}
    assert got[2][1] == "Left" and float(got[2][8]) >= 2.0           # seconds it drove since its previous row
    assert ticks and ticks[0] == ["Left"]
    assert r.st.get("charge_levels") == {}


def test_a_resumed_battery_csv_without_moved_s_keeps_its_columns(tmp_path):
    d = tmp_path / "e02-20261025-160000-dryrun"
    d.mkdir()
    (d / "battery.csv").write_text("t,robot,role,state,still_s,battery,phase,source\n")
    r = _fake(str(d), [])
    now = time.time()
    r.link = types.SimpleNamespace(robots=lambda: ["Left"], send_name=lambda n, c: None)
    r.listener = types.SimpleNamespace(battery={"Left": (now, 80)})
    r.leg_live, r.moves, r.hold, r.phase = {}, {}, threading.Event(), "route"
    r.charge_levels_tick = lambda names: None
    E.Runner.start_battery_watch(r, ["Left"])
    try:
        for _ in range(60):
            if len(open(d / "battery.csv").read().splitlines()) >= 2:
                break
            time.sleep(0.1)
    finally:
        r._bat_stop = True
    assert [len(ln.split(",")) for ln in open(d / "battery.csv").read().splitlines()] == [8, 8]


def test_robot_cards_show_the_dock_levels(tmp_path):
    import test_e02_console as TC
    r = TC.make_runner(str(tmp_path / "e02-20261025-101500-dryrun"))
    r.st["charge_levels"] = {"Left": dict(go=48.2, leave=80.0, topup=60.0, still=14.4, moving=17.9,
                                          still_from="earlier sessions (4)", rest=False)}
    v = V.LiveView(r, port=0)
    cards = {q["name"]: q for q in v.status()["live"]["robots"]}
    assert cards["Left"]["charge"]["go"] == 48.2 and cards["Left"]["charge"]["leave"] == 80.0
    assert cards["Center"]["charge"] is None
    assert "dock: go at" in V.PAGE
