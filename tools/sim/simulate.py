#!/usr/bin/env python3
"""Simulate an E1 recording session: Alvik robots driving the E1 routes, the
laptop console's log folder, the overhead camera's recording, the filled-in
tape sheet, and the ground truth the real setup can never give you.

    python tools/sim/simulate.py --out sim_out
    python tools/sim/simulate.py --out sim_out --video --seed 3
    python tools/sim/simulate.py --out sim_out --robots Left,Center,Right --video
    python tools/sim/simulate.py --out sim_out --kidnap 5          # robot picked up mid-run in sheet row 5
    python tools/sim/simulate.py --write-config my_params.json      # every assumed number, to edit

Writes, under --out:
  session/                 console log folder, exactly as tools/alvik_console.py writes it
  recording/               (with --video) camrec.py recording: video.avi, frames.csv, events.csv, meta.json
  measurements.csv         the tape sheet, filled in (start and end marks with measuring noise, run_id)
  truth.csv                true pose of every robot during every run, 50 per second, floor coordinates
  camera_truth.csv         (with --video) where each robot really is in every frame, in pixels
  camera_detections.csv    (with --video) a pretend detector's output: noisy, anonymous, with misses
  floor_calibration.json   floor cm -> image px mapping used by the camera
  sim_info.json            each robot's true wheel sizes, gyro errors, clock drift, and every run's true poses
  console_output.txt       what the console would have printed

Runs follow the sheet: one row = one run, in sheet order, with that row's
robot and commands (straight, demo and survey runs; a calibration run comes
first for each robot). Optional sheet columns start_x_cm, start_y_cm,
start_heading_deg put that row's start spot somewhere else than the session's
start_pose (e.g. a survey started in the middle of the camera view). Robots take turns (one drives while the others are parked
out of view). With --robots, only those robots' rows are used, and a listed
robot that has no rows in the sheet gets a copy of the first robot's rows.

Needs numpy and opencv-python (pip install numpy opencv-python).
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from config import REPO, dump_config, firmware_params, firmware_version, load_config  # noqa: E402
from robot import Robot, wrap180  # noqa: E402
import raw_units  # noqa: E402  (tools/raw_units.py; robot.py puts tools/ on the path)
import network  # noqa: E402

SHEET_COLS = ["order", "robot", "level", "commands", "run_id", "target_x_cm", "target_y_cm", "target_heading_deg",
              "xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm", "xf_cm", "yf_cm", "xr_cm", "yr_cm", "battery_pct", "excluded", "note"]
DEFAULT_SHEET = os.path.join(REPO, "docs", "experiments", "E01-baseline-dataset", "data", "measurements.csv")


# ------------------------------------------------------------------ sheet
def read_sheet(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def plan_rows(sheet, robots_arg):
    """Which runs to do, in order (see the module help for --robots)."""
    if not robots_arg:
        return [dict(r) for r in sheet]
    listed = [r.strip() for r in robots_arg.split(",") if r.strip()]
    rows = [dict(r) for r in sheet if r["robot"] in listed]
    first = sheet[0]["robot"]
    template = [r for r in sheet if r["robot"] == first]
    for name in listed:
        if not any(r["robot"] == name for r in rows):
            for t in template:
                c = dict(t)
                c["robot"] = name
                rows.append(c)
    for i, r in enumerate(rows, 1):
        r["order"] = str(i)
    return rows


def body_marks(x, y, th_rad, cfg, rng, noise=True):
    """Front and rear tape marks (x, y each) for a robot whose axle midpoint is at x, y."""
    m, s = cfg.robot, cfg.session
    bx = x + m.axle_to_center_cm * math.cos(th_rad)
    by = y + m.axle_to_center_cm * math.sin(th_rad)
    h = m.length_cm / 2
    pts = [bx + h * math.cos(th_rad), by + h * math.sin(th_rad), bx - h * math.cos(th_rad), by - h * math.sin(th_rad)]
    if noise:
        pts = [p + rng.gauss(0, s.tape_noise_cm) for p in pts]
    q = s.tape_round_cm
    return [("%.1f" % (round(p / q) * q)) for p in pts]


def map_polys(path):
    """Outlines of a map.json's boards (rectangles: centre line +/- thickness/2) and obstacles, as fleet_sim.py."""
    fleet = os.path.join(REPO, "tools", "fleet")
    if fleet not in sys.path:
        sys.path.insert(0, fleet)
    import fleetmap as FM
    import geometry as G
    m = FM.load(path)
    polys = [G.segment_rect(b["p1"], b["p2"], b["thickness_cm"]).tolist() for b in m["boards"]]
    for o in m["obstacles"]:
        if o["shape"] == "polygon":
            polys.append([list(p) for p in o["points"]])
        else:
            c, r = o["center"], o["radius"]
            polys.append([[c[0] + r * math.cos(a), c[1] + r * math.sin(a)] for a in np.linspace(0, 2 * np.pi, 16, endpoint=False)])
    return polys


# ------------------------------------------------------------------ main
def set_lights(rb, lc):
    """cfg.lights -> what this robot wears and how its firmware is set: l3 = holder variant (hardware) and
    lights_seq (firmware); strip = lights_strip_code. Robots missing from cfg.lights.names use the default
    table (tools/lights/patterns.py), or the firmware's own choice (strip: by name)."""
    import camera as CAM                            # puts tools/lights on the path
    LP = CAM.LP
    names = lc.names or LP.parse_names("", LP.family(lc.mode, **lc.family))
    code = names.get(rb.name)
    if lc.mode == "l3":
        rb.params["lights_mode"] = 1.0
        g, q = LP.L3Family.parse(code) if code else ("S0L", "1111")
        rb.holder = g
        rb.params["lights_seq"] = float(LP.SEQS.index(q))
    elif lc.mode == "strip":
        rb.params["lights_mode"] = 2.0
        rb.params["lights_strip_code"] = float(LP.StripFamily.parse(code)) if code else 0.0
    else:
        raise ValueError("lights.mode must be off, l3 or strip")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", help="output folder")
    ap.add_argument("--robots", help="comma-separated robot names, e.g. Left,Center,Right")
    ap.add_argument("--sheet", default=DEFAULT_SHEET, help="tape sheet giving the run order (default: E01's)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--video", action="store_true", help="also simulate the overhead camera recording")
    ap.add_argument("--config", help="JSON file overriding any assumed number (see --write-config)")
    ap.add_argument("--write-config", metavar="PATH", help="write every setting with its default value, then exit")
    ap.add_argument("--kidnap", type=int, metavar="ORDER", help="pick the robot up mid-run in this sheet row")
    ap.add_argument("--fps", type=float, help="camera frame rate (overrides the config; lower = faster)")
    ap.add_argument("--max-runs", type=int, help="only the first N runs (quick tests)")
    ap.add_argument("--map", help="tools/fleet map.json: its boards and obstacles stand on the floor during every run "
                    "(the front distance sensor's RAW dL..dR see them, as in fleet_sim.py)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.write_config:
        dump_config(cfg, args.write_config)
        print(f"wrote {args.write_config}")
        return 0
    if not args.out:
        ap.error("--out is required")
    if args.kidnap is not None:
        cfg.kidnap.run_order = args.kidnap
    if args.fps:
        cfg.camera.fps = args.fps
    t_wall0 = time.time()

    rng = random.Random(args.seed)
    nrng = np.random.default_rng(args.seed)
    out = args.out
    sess_dir = os.path.join(out, "session")
    rec_dir = os.path.join(out, "recording")
    os.makedirs(out, exist_ok=True)
    if os.path.isdir(sess_dir) and os.listdir(sess_dir):
        sys.exit(f"{sess_dir} already has files: pick a new --out folder")

    params0 = firmware_params()
    fw = firmware_version()
    rows = plan_rows(read_sheet(args.sheet), args.robots)
    if args.max_runs:
        rows = rows[:args.max_runs]
    s = cfg.session
    start_unix = dt.datetime.fromisoformat(s.start_iso).timestamp()

    # ---- the robots, each switched on some minutes before the session ----
    robots = {}
    boots = set()
    for r in rows:
        name = r["robot"]
        if name in robots:
            continue
        boot = rng.randrange(0x1000, 0x10000)
        while boot in boots:
            boot = rng.randrange(0x1000, 0x10000)
        boots.add(boot)
        t_boot = start_unix - rng.uniform(*cfg.net.boot_before_s)
        rb = Robot(name, cfg, random.Random(rng.random()), params0, fw, t_boot, boot, f"192.168.1.{40 + len(robots)}")
        if cfg.lights.mode != "off":
            set_lights(rb, cfg.lights)
        rb.t_hello = start_unix + rng.uniform(0.3, 1.5)
        rb.idle_until(rb.t_hello)
        rb.send("M,hello from %s at %s (type help in the console)" % (name, rb.ip))
        robots[name] = rb
    if args.map:
        polys = map_polys(args.map)
        for rb in robots.values():
            rb.world_polys = polys
        print(f"map {args.map}: {len(polys)} boards / obstacles in front of the distance sensor")
    print(f"simulating {len(rows)} runs on {', '.join(robots)} (firmware {fw}, seed {args.seed})")

    # ---- the session, run by run ----
    rec_start = start_unix + 4.0
    cursor = rec_start + s.video_lead_s
    prev = None
    carries = []          # (robot name, t0, t1)
    events = []           # (laptop time, text) typed into the recorder
    done = []
    calib_runs = []
    calibrated = set()

    def calibrate(r, t_lift):
        """run_sheet.py: robot put down in view, off the start mark; `calibrate`; wait for its RUNEND.
        Returns the time the robot can be picked up again for the start mark."""
        for _ in range(3):                     # run_sheet.py asks again if it didn't finish
            a = s.calib_area_cm
            cpose = (rng.uniform(a[0], a[1]), rng.uniform(a[2], a[3]), rng.uniform(-180, 180))
            r.carry_to(cpose, t_lift, t_lift + s.carry_s)
            carries.append((r.name, t_lift, t_lift + s.carry_s))
            t_cmd = t_lift + s.carry_s + s.calib_wait_s
            arrive = t_cmd + (cfg.net.oneway_base_ms + rng.expovariate(1 / cfg.net.oneway_exp_ms)) / 1000
            r.idle_until(arrive + rng.uniform(0, 0.012))
            start_true = (r.x, r.y, r.th)
            for c in s.calib_commands:          # e.g. the laptop's roll caps (set calib_dir_limits ...)
                r.receive_command(c)
            run = r.run_experiment("calib")
            t_end = r.t_at(r.ms)
            run.update(order=None, robot=r.name, row=None, start_true=start_true, end_true=(r.x, r.y, r.th),
                       t_start=r.t_at(run["ms_start"]), t_end=t_end, kidnapped=False, look=r.calib_look)
            calib_runs.append(run)
            print(f"      {r.name:7s} {'calibration':12s} run {run['run']:8s} {run['result']:22s} "
                  f"{run['t_end'] - run['t_start']:5.1f} s")
            t_lift = t_end + s.after_calib_s
            if run["result"] == "done":
                events.append((t_end + 0.05, f"calibration {r.name} run {run['run']}"))
                break
        return t_lift

    for row in rows:
        r = robots[row["robot"]]
        order = int(row["order"])
        t_lift = cursor
        if prev is not None and prev is not r:
            prev.carry_to(s.parked_pose, t_lift, t_lift + s.carry_s)
            carries.append((prev.name, t_lift, t_lift + s.carry_s))
            t_lift += s.carry_s + 1.0
        if r.name not in calibrated:
            t_lift = calibrate(r, t_lift)
            calibrated.add(r.name)
        sx, sy, sh = s.start_pose
        if (row.get("start_x_cm") or "").strip():          # sim-only sheet columns: this row's start spot
            sx, sy = float(row["start_x_cm"]), float(row["start_y_cm"])
            sh = float(row.get("start_heading_deg") or 0.0)
        pose = (sx + rng.uniform(-s.placement_err_cm, s.placement_err_cm),
                sy + rng.uniform(-s.placement_err_cm, s.placement_err_cm),
                sh + rng.uniform(-s.placement_err_deg, s.placement_err_deg))
        r.carry_to(pose, t_lift, t_lift + s.carry_s)
        carries.append((r.name, t_lift, t_lift + s.carry_s))
        t_placed = t_lift + s.carry_s
        start_true = (r.x, r.y, r.th)
        events.append((t_placed + s.before_run_s * 0.6, f"trial {order}"))

        cmds = [c.strip() for c in row["commands"].split("/") if c.strip()]
        kid = cfg.kidnap.run_order is not None and order == cfg.kidnap.run_order
        if kid and cfg.kidnap.mode == "pause":
            cmds.insert(len(cmds) - 1, "set safe_lift 0")      # otherwise the firmware stops the run when lifted
            row["commands"] = " / ".join(cmds)
        run_cmd = [c for c in cmds if c.startswith("run ")]
        if not run_cmd:
            print(f"  row {order}: no 'run' command, skipped")
            continue
        t_cmd = t_placed + s.before_run_s
        for c in cmds:
            arrive = t_cmd + (cfg.net.oneway_base_ms + rng.expovariate(1 / cfg.net.oneway_exp_ms)) / 1000
            r.idle_until(arrive + rng.uniform(0, 0.012))
            if c.startswith("run "):
                break
            r.receive_command(c)
            t_cmd += s.cmd_spacing_s
        if kid:
            r.kidnap_plan = dict(at_s=cfg.kidnap.at_s, lifted_s=cfg.kidnap.lifted_s, move_cm=cfg.kidnap.move_cm,
                                 turn_deg=cfg.kidnap.turn_deg, mode=cfg.kidnap.mode)
        run = r.run_experiment(run_cmd[0].split()[1])
        r.kidnap_plan = None
        if run is None:
            print(f"  row {order}: the robot refused to start it (not calibrated), skipped")
            cursor = r.t_at(r.ms) + s.after_run_s
            prev = r
            continue
        t_end = r.t_at(r.ms)
        run.update(order=order, robot=r.name, row=row, start_true=start_true, end_true=(r.x, r.y, r.th),
                   t_start=r.t_at(run["ms_start"]), t_end=t_end, kidnapped=kid)
        done.append(run)
        print(f"  {order:3d} {r.name:7s} {row['level']:12s} run {run['run']:8s} {run['result']:22s} "
              f"{run['t_end'] - run['t_start']:5.1f} s")
        cursor = t_end + s.after_run_s
        prev = r
    t_session_end = cursor + s.video_tail_s
    for r in robots.values():
        r.idle_until(t_session_end + 1)

    # ---- network: clock checks, delays, losses; the console writes the files ----
    clock_sends = network.add_clock_checks(list(robots.values()), start_unix + 1.0, t_session_end, cfg, rng)
    packets = []
    for r in robots.values():
        packets += network.to_packets(r, cfg, rng)
    network.write_session_json(sess_dir, start_unix, network.iso_from_unix(start_unix, s.start_iso))
    network.replay_console(sess_dir, packets, clock_sends, os.path.join(out, "console_output.txt"))

    # ---- truth, 50 per second, during every run ----
    from camera import RobotTrack, CameraGeometry   # camera needs opencv; imported here
    tracks = {n: RobotTrack(r, cfg.robot) for n, r in robots.items()}
    with open(os.path.join(out, "truth.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_laptop", "robot", "run", "x_cm", "y_cm", "heading_deg", "lifted"])
        for run in sorted(done + calib_runs, key=lambda q: q["t_start"]):
            tr = tracks[run["robot"]]
            for t in np.arange(run["t_start"], run["t_end"], 0.02):
                x, y, th, lifted = tr.pose(t)
                w.writerow([f"{t:.4f}", run["robot"], run["run"], f"{x:.3f}", f"{y:.3f}",
                            f"{wrap180(math.degrees(th)):.3f}", int(lifted)])

    # ---- the tape sheet, filled in ----
    with open(os.path.join(out, "measurements.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SHEET_COLS, extrasaction="ignore")
        w.writeheader()
        for k, run in enumerate(done):
            row = dict(run["row"])
            row["run_id"] = run["run"]
            row["xf0_cm"], row["yf0_cm"], row["xr0_cm"], row["yr0_cm"] = body_marks(*run["start_true"], cfg, rng)
            row["xf_cm"], row["yf_cm"], row["xr_cm"], row["yr_cm"] = body_marks(*run["end_true"], cfg, rng)
            row["battery_pct"] = str(run["battery"]) if (k + 1) % max(1, s.battery_every) == 0 else ""
            note = "simulated"
            if run["kidnapped"]:
                note += "; robot picked up and moved mid-run (kidnap test)"
            if run["result"].startswith("limited:"):
                # run_sheet.py, the user confirming: excluded, "limited: plywood edge" (not a failure)
                row["excluded"] = "1"
                note += "; limited: " + run["result"][len("limited:"):]
            elif run["result"] != "done":
                note += "; run ended: " + run["result"]
            row["note"] = note
            w.writerow(row)

    # ---- camera ----
    geom = CameraGeometry(cfg.camera)
    with open(os.path.join(out, "floor_calibration.json"), "w") as f:
        json.dump(geom.calibration(), f, indent=2)
    n_frames = 0
    if args.video:
        import camera
        rec_end = t_session_end - 1.0
        hands, boxes = [], []
        m = cfg.robot
        if cfg.camera.hand_on_carry:
            def follow(tr):
                return lambda t: tr.pose(t)[:2]
            for name, t0, t1 in carries:
                hands.append(camera.Hand(t0, t1, follow(tracks[name]), angle=rng.uniform(-25, 25), z=m.height_cm + 10))
            for r in robots.values():
                for k in r.kidnaps:
                    hands.append(camera.Hand(r.t_at(k["lift_ms"]), r.t_at(k["drop_ms"]), follow(tracks[r.name]),
                                             angle=rng.uniform(-25, 25), z=m.height_cm + 10))
        t = rec_start
        while cfg.camera.hands_per_min > 0:
            t += rng.expovariate(cfg.camera.hands_per_min / 60)
            if t > rec_end:
                break
            px, py = rng.uniform(-10, 130), rng.uniform(-40, 60)
            hands.append(camera.Hand(t, t + rng.uniform(0.3, 1.2), (lambda a, b: (lambda _t: (a, b)))(px, py),
                                     enter_s=0.5, angle=rng.uniform(-30, 30), z=rng.uniform(15, 30)))
        t = rec_start
        while cfg.camera.boxes_per_10min > 0:
            t += rng.expovariate(cfg.camera.boxes_per_10min / 600)
            if t > rec_end:
                break
            bx = rng.uniform(-10, 130)
            by = rng.choice([rng.uniform(-48, -42), rng.uniform(62, 68)])
            b = camera.Box(t, t + rng.uniform(20, 60), bx, by, rng.uniform(0, math.pi))
            boxes.append(b)
            for tt in (b.t0, b.t1):
                hands.append(camera.Hand(tt - 0.3, tt + 0.3, (lambda a, c: (lambda _t: (a, c)))(bx, by),
                                         enter_s=0.6, angle=rng.uniform(-20, 20), z=22))

        def progress(i, n):
            print(f"  video: frame {i} of {n}")
        print(f"rendering video ({(rec_end - rec_start) / 60:.1f} min at {cfg.camera.fps:g} fps)...")
        rec_iso = network.iso_from_unix(rec_start + cfg.camera.pi_clock_offset_s, s.start_iso)
        n_frames = camera.record(cfg, rec_dir, list(tracks.values()), hands, boxes, rec_start, rec_end, events,
                                 nrng, rec_iso, out, progress)

    # ---- what was simulated, for checking analysis results against ----
    info = dict(seed=args.seed, firmware=fw, start_unix=start_unix, frames=n_frames, robots={}, runs=[],
                raw_units=raw_units.RAW_UNITS)   # RAW speed columns in the robots' units (tools/raw_units.py)
    for n, r in robots.items():
        info["robots"][n] = dict(
            boot_id="%04X" % r.boot_id, t_boot=r.t_boot, clock_drift_ppm=r.drift_ppm,
            wheel_radius_left_cm=r.rL, wheel_radius_right_cm=r.rR, wheel_track_cm=r.track,
            nominal_radius_cm=cfg.robot.wheel_radius_cm, nominal_track_cm=cfg.robot.wheel_track_cm,
            gyro_scale_err=r.gyro_scale, gyro_bias_dps_at_end=r.gyro_bias, packets_sent=r.packets_sent,
            packets_lost=r.lost, kidnaps=r.kidnaps, blink_code=r.blink_code_for())
    def run_info(run):
        r = robots[run["robot"]]
        return dict(order=run["order"], robot=run["robot"], run=run["run"], exp=run["exp"],
                    level=run["row"]["level"] if run["row"] else "calibration", result=run["result"],
                    t_start=run["t_start"], t_end=run["t_end"],
                    start=[run["start_true"][0], run["start_true"][1], wrap180(math.degrees(run["start_true"][2]))],
                    end=[run["end_true"][0], run["end_true"][1], wrap180(math.degrees(run["end_true"][2]))],
                    turned_deg=math.degrees(run["end_true"][2] - run["start_true"][2]),
                    sync=[dict(tag=tg, ms_on=a, ms_off=b) for tg, a, b in run["syncs"]],
                    blink_code=run["blink_code"],
                    blink_start=r.t_at(run["ms_blink"]) if run["ms_blink"] is not None else None)
    for run in done:
        info["runs"].append(run_info(run))
    info["calib_runs"] = [dict(run_info(run), look=run.get("look")) for run in calib_runs]
    info["note"] = ("start/end are the TRUE axle-midpoint poses (x_cm, y_cm, heading_deg); turned_deg = true rotation "
                    "during the run, not wrapped. Robot clock: laptop_time = t_boot + robot_ms / 1000 / (1 + clock_drift_ppm "
                    "* 1e-6), so a positive clock_drift_ppm is a robot clock running FAST, which check_session.py "
                    "reports the same way (\"N ppm fast\"). calib_runs: the calibration run before each robot's first sheet "
                    "row (not on the sheet); its `look`: what the look-around chose (CALIB_LOOK; h0 = the gyro heading the "
                    "look's headings are relative to). blink_start: laptop time of blink::start(); 300 ms dark, then one 100 ms "
                    "flash every (blink_code + 2) x 100 ms, counted from blink_start + 0.3 s.")
    from dataclasses import asdict
    info["config"] = asdict(cfg)
    with open(os.path.join(out, "sim_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(f"done in {time.time() - t_wall0:.0f} s: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
