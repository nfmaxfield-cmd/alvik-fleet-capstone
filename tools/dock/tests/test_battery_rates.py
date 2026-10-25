"""Battery drain learned from the logs (battery_rates.py) and the go / leave levels it feeds (charge_levels.py)."""
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import battery_rates as BR      # noqa: E402
import charge_levels as CL      # noqa: E402

T0 = 1792884926.0


def session(name, t0, hours, robots, still=15.0, moving=25.0, asleep=None, start=95.0, move_every_min=10,
            move_s=150, seed=0, charge_at_h=None):
    """A session dict as load_session() returns: whole-% readings every 20 s, the robot driving move_s seconds every
    move_every_min minutes (or asleep the whole time if asleep is given)."""
    rng = random.Random(seed)
    rd, mov, slp = {}, {}, {}
    for r in robots:
        b, t = start, t0
        rd[r], mov[r], slp[r] = [], [], []
        if asleep is not None:
            slp[r].append((t0, float("inf")))
        nxt = t0 + 60 * move_every_min
        while t < t0 + hours * 3600:
            dt = 20.0
            if asleep is not None:
                b -= asleep * dt / 3600
            elif nxt <= t < nxt + move_s:
                b -= moving * dt / 3600
                if not mov[r] or mov[r][-1][1] < t - 1:
                    mov[r].append((t, t + move_s))
            else:
                b -= still * dt / 3600
                if t >= nxt + move_s:
                    nxt += 60 * move_every_min
            if charge_at_h is not None and abs(t - (t0 + charge_at_h * 3600)) < 10:
                b = 95.0
            t += dt
            rd[r].append((t, float(round(b + rng.uniform(-0.3, 0.3)))))
    return dict(name=name, readings=rd, charging={}, moving=mov, asleep=slp, t0=t0, t1=t0 + hours * 3600, sources=[])


def test_still_and_moving_rates_are_recovered_with_honest_errors():
    s = session("e02-a", T0, 3.0, ["Left"], still=14.0, moving=26.0)
    r, info = BR.fit_sessions([s])["Left"]
    assert abs(r["still"].rate - 14.0) < 1.5 and abs(r["moving"].rate - 26.0) < 6.0
    assert abs(r["still"].rate - 14.0) < 3 * r["still"].se + 0.5
    assert r["asleep"] is None and 0.2 < info["moving_share"] < 0.3


def test_a_charge_starts_a_new_segment():
    s = session("e02-b", T0, 4.0, ["Left"], still=15.0, moving=15.0, charge_at_h=2.0)
    smp = BR.samples([s], "Left")
    assert len(set(x[0] for x in smp)) == 2
    r, _ = BR.fit_sessions([s])["Left"]
    assert abs(r["still"].rate - 15.0) < 1.5


def test_sleep_rate_from_a_sleeping_robot():
    s = session("e02-c", T0, 3.0, ["Right"], asleep=3.0, start=80.0)
    r, _ = BR.fit_sessions([s])["Right"]
    assert r["asleep"] is not None and abs(r["asleep"].rate - 3.0) < 1.0 and r["still"] is None


def test_every_session_starts_its_own_segment():
    # 2026-10-25 (item 4): joining close sessions counted the gap as standing; the old rule is kept for --diagnose only
    a = session("e02-d1", T0, 1.0, ["Left"], start=90.0)
    b = session("e02-d2", T0 + 1.25 * 3600, 1.0, ["Left"], start=72.0)        # 15 min later, lower
    c = session("e02-d3", T0 + 30 * 3600, 1.0, ["Left"], start=90.0)          # next day
    assert len(set(x[0] for x in BR.samples([a, b, c], "Left"))) == 3
    assert len(set(x[0] for x in BR.samples([a, b, c], "Left", join=True))) == 2


def test_fallbacks_and_a_changed_robot_is_flagged(tmp_path):
    p = str(tmp_path / "bat.json")
    m = BR.BatteryModel(p)
    r = m.rates("Center")
    assert r["still"].source == "fleet default" and r["still"].rate == BR.DEFAULTS["still"][0]
    for i in range(3):          # three normal sessions
        m.update(session("e02-n%d" % i, T0 + i * 86400, 2.0, ["Center"], still=13.0, moving=20.0, seed=i))
        m.save()
    m2 = BR.BatteryModel(p)
    assert m2.rates("Center")["still"].source.startswith("earlier sessions")
    assert abs(m2.rates("Center")["still"].rate - 13.0) < 1.5
    short = session("e02-s", T0 + 5 * 86400, 0.3, ["Center"], still=22.0, seed=9)
    assert m2.update(short) == []                         # too little yet: history still used
    assert m2.rates("Center")["still"].source.startswith("earlier")
    m3 = BR.BatteryModel(p)
    words = m3.update(session("e02-x", T0 + 6 * 86400, 2.0, ["Center"], still=22.0, moving=30.0, seed=7))
    assert any("faster while still" in w for w in words)
    assert m3.rates("Center")["still"].source == "this session"
    assert m3.update(session("e02-x", T0 + 6 * 86400, 2.5, ["Center"], still=22.0, seed=7)) == []   # once only


def test_load_session_reads_battery_csv_raw_rows_and_the_sleep_line(tmp_path):
    d = tmp_path / "e02-20261024-193526"
    (d / "robot-logs-1").mkdir(parents=True)
    t0 = BR.session_start(str(d))
    lines = ["t,robot,role,state,still_s,battery,phase,source"]
    for i in range(30):
        lines.append("%.1f,Left,driving,still,300,%d,route,status" % (t0 + 300 * i, 90 - i))
        lines.append("%.1f,Right,idle,still,300,%d,setup,status" % (t0 + 300 * i, 80 - i // 6))
    (d / "battery.csv").write_text("\n".join(lines) + "\n")
    raw = ["robot,t_recv,run,wl_rpm,wr_rpm,battery,charging"]
    for k in range(100):
        raw.append("Left,%.2f,E1,40,40,89,0" % (t0 + 600 + 0.1 * k))
    (d / "robot-logs-1" / "raw.csv").write_text("\n".join(raw) + "\n")
    (d / "e02_log.txt").write_text("[19:35:40]   Right is not in this session: sent to sleep (it checks in every 10 min)\n")
    s = BR.load_session(str(d))
    assert len(s["readings"]["Left"]) == 130 and s["moving"]["Left"] and abs(s["moving"]["Left"][0][0] - (t0 + 600)) < 1
    assert s["asleep"]["Right"][0][0] - t0 == 14.0
    r, _ = BR.fit_sessions([s])["Right"]
    assert r["asleep"] is not None and r["still"] is None


def test_go_level_covers_the_queue_and_rises_with_it():
    m = {r: dict(still=BR.Rate(14, 1, 3, "t"), moving=BR.Rate(20, 2, 1, "t"), asleep=BR.Rate(3, 1, 2, "t"),
                 _moving_share=0.25) for r in ("A", "B", "C")}
    alone = CL.levels("A", m, fleet=[])
    one = CL.levels("A", m, fleet=[dict(robot="B", state="working")])
    two = CL.levels("A", m, fleet=[dict(robot="B", state="working"), dict(robot="C", state="working")])
    assert alone["go"] < one["go"] < two["go"] <= two["leave"] - 5
    # the go level is the root of go = parts(go)
    assert abs(sum(two["parts"].values()) - two["go"]) < 0.2
    nearly_done = CL.levels("A", m, fleet=[dict(robot="B", state="charging", reading=78)])
    just_in = CL.levels("A", m, fleet=[dict(robot="B", state="charging", reading=20)])
    assert nearly_done["go"] < just_in["go"]
    assert any("go" in w for w in two["reasons"]) and two["leave"] == CL.P["knee"]


def test_slow_charging_advises_rest_and_top_up_waits_for_a_free_dock():
    m = {r: dict(still=BR.Rate(15, 1, 3, "t"), moving=BR.Rate(20, 2, 1, "t"), asleep=BR.Rate(3, 1, 2, "t"),
                 _moving_share=0.25) for r in ("A", "B", "C")}
    fleet = [dict(robot="B", state="working"), dict(robot="C", state="working")]
    assert not CL.levels("A", m, fleet)["rest_advice"]
    assert CL.levels("A", m, fleet, p=dict(cc_rate=20.0))["rest_advice"]
    go, why = CL.should_go("A", 55, m, fleet=fleet)
    assert go and "top-up" in why
    go, why = CL.should_go("A", 55, m, fleet=[dict(robot="B", state="charging", reading=60), fleet[1]])
    assert not go


def test_one_still_rate_for_all_sessions_misleads_and_a_base_per_session_does_not():
    """Item 4 on synthetic data with the logs' shape: the robot drives ~20 % of every session and its standing drain
    differs from session to session; driving costs 4 %/h more than standing. One shared still rate pushes the
    sessions' differences into the moving rate; a base per session recovers driving's extra."""
    bases = [12.0, 18.0, 14.0, 17.0]
    sess = [session("e02-m%d" % i, T0 + i * 86400, 2.0 if i == 0 else 1.0, ["Left"], still=b, moving=b + 4.0,
                    start=95.0, move_every_min=10, move_s=120, seed=i) for i, b in enumerate(bases)]
    old_still, old_moving = BR.fit_one_still_rate(BR.samples(sess, "Left"))
    r, info = BR.fit(BR.samples(sess, "Left"))
    assert abs(r["extra"].rate - 4.0) < 2.0 and abs(r["extra"].rate - 4.0) < 3 * r["extra"].se + 0.5
    assert [round(b.rate) for _, b in info["bases"]] == [12, 18, 14, 17]
    assert r["moving"].rate > r["still"].rate
    assert abs((old_moving.rate - old_still.rate) - 4.0) > abs(r["extra"].rate - 4.0) + 1.0


def test_moving_follows_this_sessions_still_plus_the_learned_extra(tmp_path):
    p = str(tmp_path / "bat.json")
    m = BR.BatteryModel(p)
    for i in range(3):
        m.update(session("e02-e%d" % i, T0 + i * 86400, 2.0, ["Left"], still=14.0, moving=19.0, seed=i))
        m.save()
    m2 = BR.BatteryModel(p)
    m2.update(session("e02-e9", T0 + 9 * 86400, 1.0, ["Left"], still=20.0, moving=25.0, move_every_min=60, seed=4))
    r = m2.rates("Left")
    assert r["still"].source == "this session" and abs(r["still"].rate - 20.0) < 2.0
    assert abs(r["extra"].rate - 5.0) < 2.5 and abs(r["moving"].rate - (r["still"].rate + r["extra"].rate)) < 1e-9


def test_old_history_entries_without_extra_still_give_one():
    m = BR.BatteryModel(None)
    m.history = {"Left": [dict(id="a", still=[14.0, 1.0, 2.0], moving=[19.0, 2.0, 0.5]),
                          dict(id="b", still=[15.0, 1.0, 2.0], moving=[19.5, 2.0, 0.5])]}
    ex = m.extra("Left")
    assert 4.0 < ex.rate < 5.0 and ex.source.startswith("2 sessions")


def charging_session(name, t0, cc=30.0, knee=80.0, start=30.0, hours=3.0, robot="Center"):
    rd, chg = [], []
    b, t = start, t0
    while t < t0 + hours * 3600 and b < 99.5:
        r = cc if b < knee else cc * (100.0 - b) / (100.0 - knee)
        b += r * 20.0 / 3600.0
        t += 20.0
        rd.append((t, float(round(b))))
        chg.append(t)
    return dict(name=name, readings={robot: rd}, charging={robot: chg}, moving={}, asleep={}, t0=t0, t1=t,
                sources=[])


def test_the_charge_curve_is_learned_and_the_levels_use_it(tmp_path):
    c = BR.learn_charge(BR.charge_stretches(charging_session("e02-c1", T0), "Center"))
    assert abs(c["cc_rate"] - 30.0) < 3.0 and c["knee"] is not None and 75.0 <= c["knee"] <= 85.0
    m = BR.BatteryModel(str(tmp_path / "b.json"))
    m.update(charging_session("e02-c1", T0))
    cur = m.charge_curve()
    assert cur is not None and abs(cur["cc_rate"] - 30.0) < 3.0
    lv = CL.levels("Center", m, fleet=[dict(robot="Left", state="working")])
    assert lv["leave"] == cur["knee"] and any("learned from" in w for w in lv["reasons"])
    placeholder = CL.levels("Center", BR.BatteryModel(None), fleet=[dict(robot="Left", state="working")])
    assert any("PLACEHOLDER" in w for w in placeholder["reasons"])
    assert lv["go"] > placeholder["go"]           # 30 %/h charges slower than the placeholder 40: wait longer


def test_live_levels_from_a_session_folder(tmp_path):
    d = tmp_path / "e02-20261025-140000-dryrun"
    d.mkdir()
    t0 = BR.session_start(str(d))
    lines = ["t,robot,role,state,still_s,battery,phase,source"]
    for i in range(40):                       # 40 rows 2 min apart: 1.3 h
        t = t0 + 120 * i
        lines.append("%.1f,Left,driving,%s,%d,%d,route,status" % (t, "moving" if i % 5 == 0 else "still", 0 if i % 5 == 0
                                                                   else 120, round(95 - 16 * i * 120 / 3600)))
        lines.append("%.1f,Right,idle,asleep,0,%d,route,status" % (t + 1, round(80 - 3 * i * 120 / 3600)))
    (d / "battery.csv").write_text("\n".join(lines) + "\n")
    hist = str(tmp_path / "hist.json")
    live = CL.LiveLevels(str(d), hist)
    out, words = live.tick(["Left"], resting=("Right",))
    assert set(out) == {"Left", "Right"} and 0 < out["Left"]["go"] < out["Left"]["leave"]
    assert out["Left"]["still_from"] == "this session" and abs(out["Left"]["still"] - 16.0) < 3.0
    assert out["Right"]["asleep_from"] == "this session" and abs(out["Right"]["asleep"] - 3.0) < 1.5
    assert any(w.startswith("Left: go to the dock at") for w in words)
    assert os.path.exists(hist)
    out2, words2 = live.tick(["Left"], resting=("Right",))
    assert not any(w.startswith("Left: go") for w in words2)       # level unchanged: not repeated


def test_moved_s_in_battery_csv_gives_the_driving_time(tmp_path):
    """The runner's moved_s column (2026-10-25): without it, battery.csv alone says only whether a robot was moving at
    each row, and the learner's half-the-interval guess misses most short drives (a dry run: 7 % driving against the
    true 22 %, standing drain read 6.9 against the simulator's 3)."""
    d = tmp_path / "e02-20261025-170000"
    d.mkdir()
    t0 = BR.session_start(str(d))
    lines, b = ["t,robot,role,state,still_s,battery,phase,source,moved_s"], 95.0
    pattern = [0, 60, 10, 90, 30, 0, 45, 120, 0, 15]          # seconds driven in each 2-minute gap, never at the row
    for i in range(90):
        mv = pattern[i % len(pattern)]
        b -= (3.0 * (120 - mv) + 20.0 * mv) / 3600.0
        lines.append("%.1f,Left,driving,still,60,%d,route,status,%d" % (t0 + 120 * (i + 1), round(b), mv))
    (d / "battery.csv").write_text("\n".join(lines) + "\n")
    r, info = BR.fit_sessions([BR.load_session(str(d), robot_logs=False)])["Left"]
    assert abs(info["moving_share"] - sum(pattern) / 1200.0) < 0.03
    assert abs(r["still"].rate - 3.0) < 1.5 and abs(r["moving"].rate - 20.0) < 5.0
