"""Charge policy for one shared self-charging dock: when should a robot go to charge, and when should it leave?

    python tools/dock/charge_policy_sim.py                 the grid with the current numbers (placeholders marked)
    python tools/dock/charge_policy_sim.py --robots 4      the same for a fleet of 4
    python tools/dock/charge_policy_sim.py --awake-dock    robots stay awake while docked (no sleep)

Plan: docs/build-plans/self-charging-dock.md ("Charge thresholds").

What it simulates (minute by minute, 7 days):
  N robots work and drain. When a robot's battery READING drops to the go level G, it finishes its current job,
  drives to the dock, waits at the staging spot if the dock is taken (first come, first served), backs in, charges
  (asleep) until its reading reaches the leave level L, backs out and drives back to work.
It reports, for every (G, L) on a grid:
  uptime    the share of robot-time spent working (not driving to the dock, queuing or charging)
  min       the lowest TRUE battery any robot reached (the reading can be off by READ_ERR)
  dock      the share of time the dock is in use
Two scenarios, and each cell shows the worse of the two minima:
  worst     every robot drains at the top of the measured range, all start full at the same moment (so all reach
            G together: the longest queue), every reading reads READ_ERR too high (so they go late)
  typical   drain rates spread over the measured range, staggered starts, random reading errors; mean of 5 seeds

Every number below is tagged MEASURED (with its source), PLACEHOLDER (to be replaced from bench test 1 or another
named test) or CHOSEN. Until bench test 1 is done, the charge curve is a placeholder and so is the answer.
"""
import argparse
import random

P = dict(
    # ---------------------------------------------------------------- drain, % of a full battery per hour
    # MEASURED: docs/build-reports/e02-oct24-night-analyses.md, section C (logs/*/battery.csv and robot logs):
    # standing still and switched on, 11-16 %/h; whole-day fits while driving 13-24 %/h; robots drove about a
    # quarter of the time. A working robot's rate = 0.75 x standing + 0.25 x driving.
    work_lo=0.75 * 11 + 0.25 * 13,      # 11.5 %/h
    work_hi=0.75 * 16 + 0.25 * 24,      # 18.0 %/h
    drive=24.0,                         # MEASURED (top of the driving fits): driving to and from the dock
    wait=16.0,                          # MEASURED (top of standing): queuing at the staging spot, awake
    sleep=3.0,                          # PLACEHOLDER: drain while asleep (firmware `sleep`); test: one robot asleep
                                        # 1 h (night analyses, what's next)
    # ---------------------------------------------------------------- charge curve (bench test 1 replaces all three)
    cc_rate=40.0,                       # PLACEHOLDER: %/h in the constant-current phase (1.0 A into the 2500 mAh
                                        # cell of tech-specs.yml; the Alvik's charge current is not published)
    knee=80.0,                          # PLACEHOLDER: % where constant current turns to constant voltage (a
                                        # typical Li-ion figure); after it the rate falls in proportion to
                                        # what is left: rate = cc_rate x (100 - soc) / (100 - knee)
    # ---------------------------------------------------------------- times, minutes
    job=191 / 60.0,                     # MEASURED: longest single run in logs/*/robot-logs-*/runend.csv (741 runs,
                                        # median 28 s, 95 % 52 s, longest 191 s). A robot at G finishes it first
    drive_min=2.0,                      # CHOSEN (estimate): to the dock from anywhere on the 4 ft board, turns and
                                        # the 20 cm back-in creep included; test: time 5 send-homes
    swap_min=1.5,                       # CHOSEN (estimate): one robot backs out, the next backs in and seats
    # ---------------------------------------------------------------- reading
    read_err=3.0,                       # PLACEHOLDER: how far the battery % reading can be from the truth. The night
                                        # analyses bound sag bias at about 1 %; the gauge's own error is not
                                        # measured. Test: readings against the USB meter's mAh over a full charge
    term=0.1,                           # PLACEHOLDER: the charger stops when the current falls to this share of
                                        # the constant-current value (C/10 is typical); then the robot leaves even
                                        # if its reading is below L
    floor=5.0,                          # CHOSEN: lowest true % allowed (a margin above the gauge's empty point,
                                        # where the robot shuts down); test: run one robot down and note where
                                        # the motors weaken or it browns out
)

DT = 0.5 / 60.0        # time step, hours (30 s)
DAYS = 7.0


def charge_rate(soc, p):
    if soc < p["knee"]:
        return p["cc_rate"]
    return p["cc_rate"] * max(100.0 - soc, 0.0) / (100.0 - p["knee"])


def simulate(n, go, leave, p, scenario, seed=0, awake_dock=False, topup=None):
    rng = random.Random(seed)
    worst = scenario == "worst"
    rate = [p["work_hi"] if worst else rng.uniform(p["work_lo"], p["work_hi"]) for _ in range(n)]
    bias = [p["read_err"] if worst else rng.uniform(-p["read_err"], p["read_err"]) for _ in range(n)]
    soc = [100.0 if worst else 100.0 - rng.uniform(0, 100.0 - leave) for _ in range(n)]
    state = ["work"] * n
    left = [0.0] * n          # hours left in the current state (finish, drive, swap, back)
    queue = []
    dock = None               # robot index on the dock (or arriving at it)
    work_t = [0.0] * n
    dock_t = 0.0
    lo = min(soc)
    t, T = 0.0, DAYS * 24.0
    reading = lambda i: max(0.0, min(100.0, round(soc[i] + bias[i])))   # the gauge: whole %, 0-100
    while t < T:
        for i in range(n):
            s = state[i]
            if s == "work":
                soc[i] -= rate[i] * DT
                work_t[i] += DT
                go_now = reading(i) <= go
                if not go_now and topup is not None and dock is None and not queue and reading(i) <= topup:
                    go_now = True      # opportunistic: the dock is free, top up early
                if go_now:
                    state[i], left[i] = "finish", (rng.uniform(0, p["job"]) if not worst else p["job"]) / 60.0
            elif s == "finish":
                soc[i] -= rate[i] * DT
                work_t[i] += DT
                left[i] -= DT
                if left[i] <= 0:
                    state[i], left[i] = "drive", p["drive_min"] / 60.0
            elif s == "drive":
                soc[i] -= p["drive"] * DT
                left[i] -= DT
                if left[i] <= 0:
                    state[i] = "queue"
                    queue.append(i)
            elif s == "queue":
                soc[i] -= p["wait"] * DT
                if dock is None and queue and queue[0] == i:
                    queue.pop(0)
                    dock = i
                    state[i], left[i] = "swap", p["swap_min"] / 60.0
            elif s == "swap":
                soc[i] -= p["drive"] * DT
                left[i] -= DT
                if left[i] <= 0:
                    state[i] = "charge"
            elif s == "charge":
                d = p["wait"] if awake_dock else p["sleep"]
                soc[i] = min(100.0, soc[i] + (charge_rate(soc[i], p) - d) * DT)
                full = charge_rate(soc[i], p) < p["term"] * p["cc_rate"]      # the charger has stopped
                if reading(i) >= leave or full:
                    state[i], left[i] = "back", p["drive_min"] / 60.0
                    dock = None
            elif s == "back":
                soc[i] -= p["drive"] * DT
                left[i] -= DT
                if left[i] <= 0:
                    state[i] = "work"
            soc[i] = max(soc[i], 0.0)       # 0 = flat: the robot has died somewhere on the board
            lo = min(lo, soc[i])
        if dock is not None:
            dock_t += DT
        t += DT
    return dict(uptime=sum(work_t) / (n * T), min=lo, dock=dock_t / T)


def cell(n, go, leave, p, awake_dock=False, topup=None):
    w = simulate(n, go, leave, p, "worst", awake_dock=awake_dock, topup=topup)
    ty = [simulate(n, go, leave, p, "typical", seed=s, awake_dock=awake_dock, topup=topup) for s in range(5)]
    up = sum(r["uptime"] for r in ty) / len(ty)
    return dict(uptime=up, uptime_worst=w["uptime"], min=min(w["min"], min(r["min"] for r in ty)),
                dock=sum(r["dock"] for r in ty) / len(ty))


def go_formula(n, leave, p):
    """The go level from its parts (worst case): reading error + job + drive + worst queue + swaps + floor.
    The worst queue: every other robot ahead, each charging from about G to L. Solved by repeating (G appears on
    both sides)."""
    g = 30.0
    for _ in range(50):
        t_charge = 0.0
        s = g - p["read_err"]
        while s < leave:
            s += (charge_rate(s, p) - p["sleep"]) * DT
            t_charge += DT
        wait_h = (n - 1) * (t_charge + p["swap_min"] / 60.0)
        parts = dict(error=p["read_err"], job=p["work_hi"] * p["job"] / 60.0,
                     drive=p["drive"] * (p["drive_min"] + p["swap_min"]) / 60.0, queue=p["wait"] * wait_h,
                     floor=p["floor"])
        g_new = sum(parts.values())
        if abs(g_new - g) < 0.05:
            break
        g = g_new
    return g_new, parts, wait_h


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=3)
    ap.add_argument("--awake-dock", action="store_true", help="robots stay awake while docked")
    a = ap.parse_args(argv)
    p, n = P, a.robots
    print("charge policy, %d robots, one dock%s" % (n, ", awake while docked" if a.awake_dock else ", asleep while docked"))
    print("PLACEHOLDERS in use (replace after bench test 1 and the sleep test): charge %.0f %%/h to a knee at %.0f %%, "
          "sleep drain %.0f %%/h, reading error %.0f %%" % (p["cc_rate"], p["knee"], p["sleep"], p["read_err"]))
    work_mid = (p["work_lo"] + p["work_hi"]) / 2
    net = p["cc_rate"] - (p["wait"] if a.awake_dock else p["sleep"])
    print("\ncapacity (no queue): one robot works %.0f %% of the time at most (charge %.0f / (charge + drain %.1f));"
          % (100 * net / (net + work_mid), net, work_mid))
    print("  one dock keeps at most %.1f robots running (1 + charge / drain); %d robots keep it busy %.0f %% of the time"
          % (1 + net / work_mid, n, 100 * n * work_mid / (net + work_mid)))
    gos = list(range(10, 65, 5))
    leaves = list(range(55, 101, 5))
    policies = [("plain: go at G, leave at L", None),
                ("plus early top-up: also go whenever the dock is free, nobody waits and the reading is below L - 20", 20),
                ("plus early top-up below L - 10", 10)]
    best_all = None
    for title, gap in policies:
        res = {}
        for g in gos:
            for L in leaves:
                if L >= g + 15:
                    res[(g, L)] = cell(n, g, L, p, a.awake_dock, topup=None if gap is None else L - gap)
        print("\n%s" % title)
        print("uptime %% (typical) / lowest true battery %% (worst of both scenarios); '*' = below the %.0f %% floor, "
              "0 = a robot went flat" % p["floor"])
        print("go\\leave " + "".join("%10d" % L for L in leaves))
        for g in gos:
            row = "%7d  " % g
            for L in leaves:
                r = res.get((g, L))
                row += "%10s" % ("" if r is None else "%2.0f/%2.0f%s" % (100 * r["uptime"], r["min"],
                                                                      "*" if r["min"] < p["floor"] else " "))
            print(row)
        safe = {k: v for k, v in res.items() if v["min"] >= p["floor"]}
        if not safe:
            print("no safe cell: one dock cannot keep this fleet above the floor with this policy")
            continue
        best = max(safe, key=lambda k: (round(safe[k]["uptime"], 3), safe[k]["min"]))
        b = safe[best]
        print("best safe pair: go %d %%, leave %d %%: uptime %.1f %% typical (%.1f %% worst case), lowest %.1f %%, "
              "dock busy %.0f %%" % (best[0], best[1], 100 * b["uptime"], 100 * b["uptime_worst"], b["min"],
                                     100 * b["dock"]))
        print("uptime against the leave level at go %d %% (the knee is at %.0f %%): %s" % (
            best[0], p["knee"], ", ".join("%d: %.1f" % (L, 100 * res[(best[0], L)]["uptime"]) for L in leaves
                                          if (best[0], L) in res)))
        if best_all is None or b["uptime"] > best_all[2]["uptime"] + 1e-3:
            best_all = (title, best, b)
    if best_all is None:
        return 1
    title, best, b = best_all
    gf, parts, wait_h = go_formula(n, best[1], p)
    print("\nbest policy: %s, go %d %%, leave %d %%: uptime %.1f %%, lowest %.1f %%" % (
        title.split(":")[0], best[0], best[1], 100 * b["uptime"], b["min"]))
    print("go level from its parts, plain policy, worst case, leave %d %%: %.1f %% = %s (worst queue %.1f h)"
          % (best[1], gf, " + ".join("%s %.1f" % (k, v) for k, v in parts.items()), wait_h))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
