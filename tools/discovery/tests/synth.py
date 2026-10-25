"""A synthetic robot for the discovery tests: a two-wheeled robot driving and turning in a 120 x 120 cm box, with
the same kinds of streams as the Alvik's RAW record (wheel angles and speeds, a 6-axis IMU, a fused orientation,
a 7-zone range sensor whose centre zone is the mean of its neighbours, 3 line sensors, battery, clock, counter,
step index, run id, unused switches) and a computed pose. Every stream's true type is known.

make_session(dir) writes a session folder in the console's layout (robot-logs-1/raw.csv, leg.csv, runinfo.csv,
track.csv, e02_state.json) so load.py reads it like a real one."""
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import raw_units                                             # noqa: E402  (tools/raw_units.py)

HZ = 50.0
R_WHEEL = 1.7       # cm
HALF_AXLE = 4.8     # cm
BOX = 120.0

COLUMNS = ["run", "ms", "step", "wl_deg", "wr_deg", "wl_rpm", "wr_rpm", "v_cms", "w_dps", "x_cm", "y_cm", "th_deg",
           "ax", "ay", "az", "gx", "gy", "gz", "roll_deg", "pitch_deg", "yaw_deg", "dL_cm", "dCL_cm", "dC_cm",
           "dCR_cm", "dR_cm", "dtop_cm", "dbot_cm", "lineL", "lineC", "lineR", "battery", "charging", "lifted",
           "swL", "swR", "raw_n"]


def _plan(rng, n_actions):
    """[(kind, value, duration s)]: rests between drives and turns."""
    out = []
    for i in range(n_actions):
        out.append(("rest", 0.0, float(rng.uniform(1.5, 3.0))))
        if rng.random() < 0.5:
            d = float(rng.choice([-1, 1]) * rng.uniform(8, 20))
            out.append(("go", d, abs(d) / 8.0))
        else:
            a = float(rng.choice([-1, 1]) * rng.uniform(40, 150))
            out.append(("turn", a, abs(a) / 40.0))
    out.append(("rest", 0.0, 2.0))
    return out


def _ray(x, y, th_deg):
    """Distance from (x, y) along heading th to the box walls."""
    a = math.radians(th_deg)
    c, s = math.cos(a), math.sin(a)
    best = 400.0
    for wall, axis in ((0.0, 0), (BOX, 0), (0.0, 1), (BOX, 1)):
        d = c if axis == 0 else s
        p = x if axis == 0 else y
        if abs(d) < 1e-6:
            continue
        t = (wall - p) / d
        if t > 0:
            best = min(best, t)
    return best


def simulate(seed=1, n_actions=60):
    rng = np.random.default_rng(seed)
    plan = _plan(rng, n_actions)
    rows, legs, cam = [], [], []
    x, y, th = 60.0, 60.0, 0.0           # true pose (deg)
    wl = wr = 0.0
    yaw_int = 0.0
    t = 1000.0
    ms0 = 123456
    n = 0
    step = 0
    battery = 97
    frame = {"yaw": 0.0, "roll": 0.0, "pitch": 0.0, "zones": [_ray(x, y, th + a) for a in (22, 11, 0, -11, -22)],
             "top": _ray(x, y, th), "bot": 7.0}
    line = [50, 52, 49]
    prev_v = 0.0
    for kind, val, dur in plan:
        nrow = max(1, int(round(dur * HZ)))
        if kind in ("go", "turn"):
            legs.append({"kind": kind, "value": val, "t0": t, "t1": t + nrow / HZ})
            step += 1
        for i in range(nrow):
            dt = 1.0 / HZ
            v = val / dur if kind == "go" else 0.0                 # cm/s
            w = val / dur if kind == "turn" else 0.0               # deg/s
            # wheels (deg) from the motion
            vl = v - math.radians(w) * HALF_AXLE
            vr = v + math.radians(w) * HALF_AXLE
            wl += math.degrees(vl * dt / R_WHEEL)
            wr += math.degrees(vr * dt / R_WHEEL)
            x += v * dt * math.cos(math.radians(th))
            y += v * dt * math.sin(math.radians(th))
            th += w * dt
            # what a robot logs (tools/raw_units.py): rpm, v_cms and w_dps 1.08 high, v_cms in mm/s; the
            # board reports rpm in steps of about 0.66
            sp = raw_units.robot_columns(vl / (2 * math.pi * R_WHEEL) * 60, vr / (2 * math.pi * R_WHEEL) * 60,
                                         R_WHEEL, 2 * HALF_AXLE)
            rpm_l = round(sp["wl_rpm"] / 0.667) * 0.667
            rpm_r = round(sp["wr_rpm"] / 0.667) * 0.667
            gz = w * 0.97 + 0.6 + rng.normal(0, 0.03)
            gx = -0.9 + rng.normal(0, 0.03)
            gy = 0.3 + rng.normal(0, 0.03)
            acc_fwd = (v - prev_v) / dt / 981.0
            prev_v = v
            ax = 0.01 + acc_fwd + rng.normal(0, 0.001)
            ay = -0.008 + rng.normal(0, 0.001)
            az = -1.0 + rng.normal(0, 0.001)
            yaw_int += gz * dt
            if n % 3 == 0:          # fused orientation on its own slower tick
                frame["yaw"] = yaw_int % 360.0
                frame["roll"] = math.degrees(math.atan2(ay, -az))
                frame["pitch"] = math.degrees(math.atan2(-ax, -az))
            if n % 3 == 1:          # range frames on another
                zones = [_ray(x, y, th + a) + rng.normal(0, 0.8) for a in (22, 11, -11, -22)]
                frame["zones"] = [zones[0], zones[1], 0.5 * (zones[1] + zones[2]), zones[2], zones[3]]
                frame["top"] = _ray(x, y, th) + rng.normal(0, 1.0)
                frame["bot"] = 7.0 + rng.normal(0, 0.05)
            if n % 4 == 0:
                line = [int(round(50 + rng.normal(0, 1))), int(round(52 + rng.normal(0, 1))),
                        int(round(49 + rng.normal(0, 1)))]
            if n and n % int(HZ * 100) == 0:
                battery -= 1
            pose_th = (wr - wl) * R_WHEEL / (2 * HALF_AXLE)       # deg, from the wheels
            rows.append({
                "run": "AB12-1", "ms": ms0 + int(n * 20 + rng.integers(0, 3)), "step": step,
                "wl_deg": "%.2f" % wl, "wr_deg": "%.2f" % wr, "wl_rpm": "%.2f" % rpm_l, "wr_rpm": "%.2f" % rpm_r,
                "v_cms": "%.2f" % ((rpm_l + rpm_r) * math.pi * R_WHEEL / 60.0 * raw_units.V_COL_PER_CMS),
                "w_dps": "%.2f" % ((rpm_r - rpm_l) * 2 * math.pi * R_WHEEL / 60.0 / (2 * HALF_AXLE) * 57.2958),
                "x_cm": "%.2f" % (x - 60.0), "y_cm": "%.2f" % (y - 60.0), "th_deg": "%.2f" % pose_th,
                "ax": "%.4f" % ax, "ay": "%.4f" % ay, "az": "%.4f" % az,
                "gx": "%.3f" % gx, "gy": "%.3f" % gy, "gz": "%.3f" % gz,
                "roll_deg": "%.2f" % frame["roll"], "pitch_deg": "%.2f" % frame["pitch"],
                "yaw_deg": "%.2f" % frame["yaw"],
                "dL_cm": "%.1f" % frame["zones"][0], "dCL_cm": "%.1f" % frame["zones"][1],
                "dC_cm": "%.1f" % frame["zones"][2], "dCR_cm": "%.1f" % frame["zones"][3],
                "dR_cm": "%.1f" % frame["zones"][4], "dtop_cm": "%.1f" % frame["top"],
                "dbot_cm": "%.1f" % frame["bot"],
                "lineL": line[0], "lineC": line[1], "lineR": line[2], "battery": battery, "charging": 0,
                "lifted": 0, "swL": -1, "swR": -1, "raw_n": n,
            })
            if n % 3 == 0:
                cam.append((t + rng.normal(0, 0.005), x + rng.normal(0, 0.3), y + rng.normal(0, 0.3)))
            t += dt
            n += 1
    return rows, legs, cam


def make_session(d, seed=1, n_actions=60, robot="Synth"):
    """Write a synthetic session folder; returns its path."""
    rows, legs, cam = simulate(seed, n_actions)
    sub = os.path.join(d, "robot-logs-1")
    os.makedirs(sub, exist_ok=True)
    t0 = 1000.0
    with open(os.path.join(sub, "raw.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv"] + COLUMNS)
        rng = np.random.default_rng(seed + 7)
        last = 0.0
        for i, r in enumerate(rows):
            # laptop arrival: send time plus a WiFi delay that is never negative; lines arrive in order
            last = max(last, t0 + i / HZ + abs(rng.normal(0, 0.01)))
            w.writerow([robot, "%.4f" % last] + [r[c] for c in COLUMNS])
    with open(os.path.join(sub, "leg.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "id", "kind", "value", "result", "ms", "x_cm", "y_cm", "th_deg", "info"])
        for i, L in enumerate(legs):
            w.writerow([robot, "%.4f" % L["t0"], "AB12-1", i, L["kind"], "%.2f" % L["value"], "started", 0, 0, 0, 0, ""])
            w.writerow([robot, "%.4f" % L["t1"], "AB12-1", i, L["kind"], "%.2f" % L["value"], "done", 0, 0, 0, 0, ""])
    with open(os.path.join(sub, "runinfo.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "run", "experiment", "fw_version"])
        w.writerow([robot, "1000.0", "AB12-1", "synth", "synthetic"])
    with open(os.path.join(d, "track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm", "order", "phase", "source", "lock_score"])
        for t, x, y in cam:
            w.writerow(["%.3f" % t, robot, "%.2f" % x, "%.2f" % y, "", "", "lock", "1"])
    with open(os.path.join(d, "sim_info.json"), "w") as f:      # simulator marker: RAW speeds in robot units
        json.dump({"source": "tools/discovery/tests/synth.py", "raw_units": raw_units.RAW_UNITS}, f)
    with open(os.path.join(d, "e02_state.json"), "w") as f:
        json.dump({"board": {"xboard_cm": BOX, "yboard_cm": BOX, "xboard_side": 1, "yboard_side": 1}}, f)
    return d
