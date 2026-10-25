"""The go and leave levels for one robot right now, from its measured drain, the dock queue and the reading error
(Nick, 2026-10-25). The runner calls levels() whenever it decides whether a robot should head for the dock.

    import battery_rates as BR, charge_levels as CL
    model = BR.BatteryModel()                     # learned rates (logs/battery_learned.json + this session)
    lv = CL.levels("Left", model, fleet=[dict(robot="Center", state="charging", reading=55),
                                         dict(robot="Right", state="working", reading=71)])
    lv["go"], lv["leave"], lv["topup"], lv["reasons"]

The go level is the plan's method (docs/build-plans/self-charging-dock.md, "charge thresholds"), with each drain now
the ROBOT'S OWN measured rate at its upper bound (rate + K_SE standard errors):
    go = reading error + finishing the job + driving to the dock and backing in + waiting for the dock + floor
The wait covers the robots ahead of it: the one on the dock (from its reading to its leave level), the ones queued
(each a whole charge), and every other WORKING robot, as if it reached its own go level at the same moment (the worst
case; it depends on the go level itself, so it is solved by repeating). A robot resting asleep off the dock is not
counted (it joins the queue only when it reaches the go level asleep, hours later).
The leave level is the charge curve's knee (constant current turns to constant voltage); the early top-up level is
leave - TOPUP_GAP (the policy simulator's best, tools/dock/charge_policy_sim.py).
A capacity check says when one dock cannot keep this many robots working (1 + net charge rate / drain): then rest one
robot at a time asleep off the dock (the simulator's `--rest` policy; asleep by the firmware's `sleep`, never switched
off by hand).

The levels ADAPT as data comes in (Nick, 2026-10-25 13:03: "as long as we adapt once we have data"): the drains are
the battery model's (this session's own once it has enough hours, else the earlier sessions', else the fleet
default), their standard errors shrink with more hours, and the charge curve (rate and knee, so the leave level too)
is the one learned from the robots' charging (BatteryModel.charge_curve) as soon as there is CHARGE_MIN_H of it.
LiveLevels below is what the runner calls every battery tick.
Every number below: MEASURED / PLACEHOLDER / CHOSEN as in the policy simulator. Tests: tools/dock/tests/.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import battery_rates as BR      # noqa: E402

P = dict(
    read_err=3.0,          # PLACEHOLDER: the gauge's error, % (bench test 1: readings against the USB meter's mAh)
    floor=5.0,             # CHOSEN: lowest true % allowed (test: run one robot down, note where it browns out)
    job_min=191 / 60.0,    # MEASURED: longest run in logs/*/robot-logs-*/runend.csv (741 runs)
    drive_min=2.0,         # CHOSEN (estimate): to the dock from anywhere on the board; timed in bench test 8
    swap_min=1.5,          # CHOSEN (estimate): one robot backs out, the next backs in
    cc_rate=40.0,          # PLACEHOLDER: %/h before the knee (bench test 1)
    knee=80.0,             # PLACEHOLDER: % where the charge rate starts to fall (bench test 1)
    term=0.1,              # PLACEHOLDER: the charger stops at this share of the constant current
    k_se=2.0,              # CHOSEN: drains are taken at rate + 2 standard errors (about 98 % one-sided). DOE note:
                           # the policy simulator with each robot's rate drawn from its uncertainty, k = 1 / 2 / 3,
                           # scored on flat batteries against uptime (doe-backlog.md)
    topup_gap=20.0,        # set by the policy simulator (best of gaps 10 and 20 at the placeholder curve)
    awake_dock=False,      # Nick 2026-10-25: robots sleep while docked
)
DT_H = 0.5 / 60.0


def charge_rate(soc, p):
    if soc < p["knee"]:
        return p["cc_rate"]
    return p["cc_rate"] * max(100.0 - soc, 0.0) / (100.0 - p["knee"])


def charge_hours(frm, to, p, dock_drain):
    """Hours on the dock from frm % to to % (or until the charger stops), net of dock_drain %/h."""
    s, h = max(0.0, frm), 0.0
    while s < to and h < 48:
        r = charge_rate(s, p)
        if r < p["term"] * p["cc_rate"] or r - dock_drain <= 0:
            break
        s += (r - dock_drain) * DT_H
        h += DT_H
    return h


def _rates(model, robot):
    if hasattr(model, "rates"):
        return model.rates(robot), model.moving_share(robot)
    r = model[robot]
    return r, r.get("_moving_share", 0.25) if isinstance(r, dict) else 0.25


def levels(robot, model, fleet=(), p=None):
    """robot's go / leave / top-up levels now. model: a battery_rates.BatteryModel (or {robot: {state: Rate}}).
    fleet: the OTHER robots, each dict(robot, state, reading): state is 'charging' (on the dock), 'queued' (waiting
    for it, in queue order), 'working', or 'resting' (asleep off the dock). Returns dict(go, leave, topup, parts,
    wait_h, rates, capacity, rest_advice, reasons)."""
    learned = model.charge_curve() if hasattr(model, "charge_curve") else None
    base = dict(P)
    if learned is not None:
        base["cc_rate"] = learned["cc_rate"]
        if learned.get("knee") is not None:
            base["knee"] = learned["knee"]
    p = dict(base, **(p or {}))
    k = p["k_se"]
    rates, share = _rates(model, robot)
    still_hi, move_hi = rates["still"].hi(k), rates["moving"].hi(k)
    dock_drain = (rates["still"] if p["awake_dock"] else rates["asleep"]).hi(k)
    work_hi = (1 - share) * still_hi + share * move_hi
    leave = p["knee"]
    reasons = []
    others = list(fleet)
    ahead_fixed_h = 0.0
    n_working = 0
    desc = []
    for o in others:
        st, rd = o.get("state"), o.get("reading")
        orates, _ = _rates(model, o["robot"]) if (hasattr(model, "rates") or o["robot"] in model) else (rates, share)
        odrain = (orates["still"] if p["awake_dock"] else orates["asleep"]).hi(k)
        if st == "charging":
            h = charge_hours((rd if rd is not None else 0.0) - p["read_err"], leave, p, odrain)
            ahead_fixed_h += h + p["swap_min"] / 60.0
            desc.append("%s on the dock at %s%%: %.1f h to %.0f %%" % (o["robot"], "?" if rd is None else "%.0f" % rd,
                                                                      h, leave))
        elif st == "queued":
            h = charge_hours((rd if rd is not None else 0.0) - p["read_err"], leave, p, odrain)
            ahead_fixed_h += h + p["swap_min"] / 60.0
            desc.append("%s queued at %s%%: %.1f h" % (o["robot"], "?" if rd is None else "%.0f" % rd, h))
        elif st == "working":
            n_working += 1
        elif st == "resting":
            desc.append("%s resting asleep off the dock: not counted" % o["robot"])
    # each working robot may reach ITS go level at the same moment as this one (the worst case), so the wait depends
    # on the go level: f(g) falls as g rises, so g = f(g) has one root; found by bisection
    def f(g):
        per = charge_hours(g - p["read_err"], leave, p, dock_drain) + p["swap_min"] / 60.0
        wait = ahead_fixed_h + n_working * per
        pt = dict(error=p["read_err"], job=move_hi * p["job_min"] / 60.0,
                  drive=move_hi * (p["drive_min"] + p["swap_min"]) / 60.0, queue=still_hi * wait, floor=p["floor"])
        return sum(pt.values()), pt, wait, per

    lo, hi = 0.0, 100.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if f(mid)[0] > mid:
            lo = mid
        else:
            hi = mid
    g_new, parts, wait_h, per = f(hi)
    g_new = hi
    go = min(g_new, leave - 5.0)
    if g_new > leave - 5.0:
        reasons.append("the worst wait alone would need a go level of %.0f %%, above the leave level: one dock cannot "
                       "cover this queue; rest a robot (rest_advice)" % g_new)
    # capacity: 1 + net charge rate / mean working drain
    net = p["cc_rate"] - dock_drain
    work_mean = (1 - share) * rates["still"].rate + share * rates["moving"].rate
    cap = 1.0 + net / max(work_mean, 0.1)
    active = 1 + n_working + sum(1 for o in others if o.get("state") in ("charging", "queued"))
    rest = active > math.floor(cap + 1e-9)
    reasons.insert(0, "drain (%s): still %s, moving %s, docked asleep %s; used at +%g se: still %.1f, moving %.1f, "
                      "docked %.1f %%/h; moving share %.2f" % (robot, rates["still"], rates["moving"], rates["asleep"],
                                                               k, still_hi, move_hi, dock_drain, share))
    reasons.append("go %.1f %% = " % go + " + ".join("%s %.1f" % (a, b) for a, b in parts.items())
                   + " (wait %.2f h: %s%s)" % (wait_h, "; ".join(desc) or "dock free",
                                                ("; %d working robot%s could arrive first, %.1f h each"
                                                 % (n_working, "" if n_working == 1 else "s", per)) if n_working else ""))
    if learned is not None:
        reasons.append("leave %.0f %% = the charge curve's knee (%.0f %%/h to %.0f %%, %s%s)" % (
            leave, p["cc_rate"], p["knee"], learned["source"],
            "" if learned.get("knee") is not None else "; knee still the PLACEHOLDER: no charge has reached it yet"))
    else:
        reasons.append("leave %.0f %% = the charge curve's knee (PLACEHOLDER %.0f %%/h to %.0f %% until the robots have "
                       "charged %.2f h on the dock, or bench test 1)" % (leave, p["cc_rate"], p["knee"], BR.CHARGE_MIN_H))
    reasons.append("capacity: one dock keeps %.1f robots working (net charge %.1f / drain %.1f %%/h); %d active%s"
                   % (cap, net, work_mean, active, ": REST one robot at a time asleep off the dock" if rest else ""))
    return dict(go=go, leave=leave, topup=leave - p["topup_gap"], parts=parts, wait_h=wait_h, capacity=cap,
                rest_advice=rest, reasons=reasons,
                rates=dict(still=still_hi, moving=move_hi, docked=dock_drain, work=work_hi, share=share))


def should_go(robot, reading, model, fleet=(), p=None):
    """(go now?, why): go at the go level; or top up early when the dock is free, nobody waits, and the reading is
    below the top-up level."""
    lv = levels(robot, model, fleet, p)
    if reading is None:
        return False, "no reading"
    if reading <= lv["go"]:
        return True, "reading %.0f %% at or below the go level %.1f %%" % (reading, lv["go"])
    busy = any(o.get("state") in ("charging", "queued") for o in fleet)
    if not busy and reading <= lv["topup"]:
        return True, "dock free and nobody waiting: early top-up (reading %.0f %% below %.0f %%)" % (reading, lv["topup"])
    return False, "reading %.0f %% above go %.1f %%%s" % (reading, lv["go"], "" if not busy else " (dock busy)")


def sim_params(model, robots):
    """The policy simulator's drain numbers from the learned rates (charge_policy_sim.P overrides)."""
    w, mv, st, sl = [], [], [], []
    for r in robots:
        rates, share = _rates(model, r)
        w.append((1 - share) * rates["still"].rate + share * rates["moving"].rate)
        mv.append(rates["moving"].rate)
        st.append(rates["still"].rate)
        sl.append(rates["asleep"].rate)
    return dict(work_lo=min(w), work_hi=max(w), drive=max(mv), wait=max(st), sleep=max(sl))


class LiveLevels:
    """The runner's hook (tools/experiment/e02.py, every battery tick): refit the battery model from the session
    folder so far (battery.csv and e02_log.txt only: cheap), save it into the history, and give every robot's go /
    leave / top-up level with where its numbers come from.

        live = LiveLevels(session_dir, history_path)
        levels, words = live.tick(robots, resting=("Right",))     # words: change messages and level moves to log
    """

    def __init__(self, session_dir, history_path, p=None):
        self.dir = session_dir
        self.model = BR.BatteryModel(history_path)
        self.p = p
        self.last = {}                 # robot -> go level last reported

    def tick(self, robots, resting=(), charging=None, queued=(), readings=None):
        """robots: the session's robots (working unless named in resting / queued, or the one charging);
        readings: {robot: latest %} (for the one on the dock). Returns ({robot: dict}, [message lines])."""
        sess = BR.load_session(self.dir, robot_logs=False)
        words = list(self.model.update(sess))
        try:
            self.model.save()
        except OSError as e:
            words.append("battery history not saved: %s" % e)
        readings = readings or {}
        out = {}
        names = list(robots) + [r for r in resting if r not in robots]

        def state(o):
            if o == charging:
                return "charging"
            if o in queued:
                return "queued"
            return "resting" if o in resting else "working"

        for r in names:
            fleet = [dict(robot=o, state=state(o), reading=readings.get(o)) for o in names if o != r]
            lv = levels(r, self.model, fleet, self.p)
            rates = self.model.rates(r)
            cur = self.model.charge_curve()
            out[r] = dict(go=round(lv["go"], 1), leave=round(lv["leave"], 1), topup=round(lv["topup"], 1),
                          still=round(rates["still"].rate, 2), still_se=round(rates["still"].se, 2),
                          moving=round(rates["moving"].rate, 2), moving_se=round(rates["moving"].se, 2),
                          asleep=round(rates["asleep"].rate, 2), still_from=rates["still"].source,
                          asleep_from=rates["asleep"].source,
                          charge_from=cur["source"] if cur else "placeholder (no charging yet)",
                          rest=bool(lv["rest_advice"]), t=round(sess["t1"] or 0.0, 1))
            prev = self.last.get(r)
            if prev is None or abs(prev - lv["go"]) >= 1.0:
                self.last[r] = lv["go"]
                words.append("%s: go to the dock at %.0f %%, leave at %.0f %% (still %.1f +- %.1f %%/h from %s; "
                             "moving %.1f; charge curve %s)" % (r, lv["go"], lv["leave"], rates["still"].rate,
                                                                rates["still"].se, rates["still"].source,
                                                                rates["moving"].rate, out[r]["charge_from"]))
        return out, words
