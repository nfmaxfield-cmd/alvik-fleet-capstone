#!/usr/bin/env python3
"""Score blink.py and calib.py against a simulator run (tools/sim/simulate.py --video).

    python tools/sim/simulate.py --out sim_out --video --seed 7 --max-runs 6
    python tools/blink/eval_sim.py sim_out            # runs blink.py and calib.py if their outputs are missing

Reports:
  - decode accuracy: of the blinking robot-seconds (seconds in which a robot
    was in a run, in view, and its lights were lit in 2+ frames), the share
    where its light was reported with its own name and never with another
  - light position error: reported light centroid vs the true midpoint of the
    two LEDs (from camera_truth.csv: top centre + the sim's LED offset along
    the heading)
  - calibration: heading error of each calib roll / start nudge vs truth.csv,
    and the LED offset found vs the sim's robot model
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tests"))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "fusion"))
import blink  # noqa: E402
import calib  # noqa: E402
import score  # noqa: E402
import runs as fr  # noqa: E402


def wrap(a):
    return (a + 180) % 360 - 180


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sim", help="simulate.py --out folder")
    ap.add_argument("--rerun", action="store_true", help="run blink.py and calib.py even if outputs exist")
    args = ap.parse_args(argv)
    S = args.sim
    rec, ses = os.path.join(S, "recording"), os.path.join(S, "session")
    info = json.load(open(os.path.join(S, "sim_info.json")))
    cfg = info["config"]
    with open(os.path.join(S, "camera_truth.csv")) as f:
        if "led_on" not in f.readline():
            sys.exit("camera_truth.csv has no led_on column: this simulator does not blink codes yet")
    calib_json = os.path.join(S, "floor_calibration.json")
    bpath = os.path.join(rec, "blink_detections.csv")
    if args.rerun or not os.path.exists(bpath):
        blink.main([rec, "--out", bpath, "--calib", calib_json])
    # ---- truth: LED midpoint per frame, lit only inside a run's blinking span
    m = cfg["robot"]
    led_fwd = m["length_cm"] / 2 - 2.0      # LED midpoint ahead of the body centre (tools/sim/camera.py)
    ppc_top = cfg["camera"]["width"] / cfg["camera"]["floor_width_cm"] * cfg["camera"]["height_cm"] / (
        cfg["camera"]["height_cm"] - m["height_cm"])
    off_px = led_fwd * ppc_top
    runs, fits = fr.load_runs(ses)
    pi_off = cfg["camera"]["pi_clock_offset_s"]
    delay = cfg["camera"]["capture_delay_frames"] / cfg["camera"]["fps"]
    spans = defaultdict(list)       # robot -> [(t0, t1)] on the frame (capture) clock, laptop time
    for run in runs:
        if "start" in run.sync and "end" in run.sync:
            spans[run.robot].append((run.sync["start"][1] + 0.3, run.sync["end"][0] - 0.15))
    truth = []
    with open(os.path.join(S, "camera_truth.csv")) as f:
        for r in csv.DictReader(f):
            t = float(r["t_wall"])
            tc = t - pi_off - delay
            h = math.radians(float(r["heading_img_deg"]))
            on = int(r["led_on"]) and any(a <= tc <= b for a, b in spans[r["robot"]])
            truth.append(dict(frame=int(r["frame"]), t=t, robot=r["robot"], led=1.0 if on else 0.0,
                              x=float(r["x_px"]) + off_px * math.cos(h), y=float(r["y_px"]) - off_px * math.sin(h)))
    rows = blink.read_blink_csv(bpath)
    res = score.evaluate(rows, truth)
    print("BLINK  " + score.describe(res))
    # ---- calibration
    cpath = os.path.join(rec, "calibration_results.csv")
    off = fr.video_offset(ses, rec, fits)        # (reads the whole video once)
    off = off[0] if off else pi_off + delay
    if args.rerun or not os.path.exists(cpath):
        calib.main(["--session", ses, "--recording", rec, "--blink", bpath, "--calib", calib_json,
                    "--detections", os.path.join(S, "camera_detections.csv"), "--out", cpath,
                    "--video-offset", str(off)])
    tt = defaultdict(list)
    with open(os.path.join(S, "truth.csv")) as f:
        for r in csv.DictReader(f):
            tt[r["run"]].append((float(r["t_laptop"]), float(r["heading_deg"])))
    errs, offs = [], []
    with open(cpath) as f:
        for r in csv.DictReader(f):
            if not r["heading_floor_deg"]:
                print(f"CALIB  {r['run']} {r['event']}: no heading ({r['note']})")
                continue
            ts = tt.get(r["run"])
            t = float(r["t_video"]) - off
            if not ts:
                # calib runs are not in truth.csv (it covers sheet runs only): use the next run's start heading
                nxt = [q for q in info["runs"] if q["robot"] == r["robot_log"] and q["t_start"] > t]
                true_h = min(nxt, key=lambda q: q["t_start"])["start"][2] if nxt else None
                src = "next run's start (robot is placed on its mark after calibrating: NOT the same pose)"
            else:
                i = bisect.bisect_left([q[0] for q in ts], t)
                true_h = ts[min(i, len(ts) - 1)][1]
                src = "truth.csv"
            e = wrap(float(r["heading_floor_deg"]) - true_h) if true_h is not None else float("nan")
            if src == "truth.csv":
                errs.append(e)
            if r["led_fwd_cm"]:
                offs.append((float(r["led_fwd_cm"]), float(r["led_left_cm"])))
            print(f"CALIB  {r['run']} {r['event']:5s} {r['robot_log']}: blink says {r['robot_blink'] or '-'} "
                  f"({r['identity']}); heading {float(r['heading_floor_deg']):7.2f}, true {true_h if true_h is None else round(true_h, 2)} "
                  f"[{src}], error {e:+.2f} deg; moved {r['moved_cm']} cm; LED offset fwd {r['led_fwd_cm']} "
                  f"left {r['led_left_cm']} cm")
            rb = info["robots"].get(r["robot_log"], {})
            if r.get("wheel_scale"):
                ws = (rb["wheel_radius_left_cm"] + rb["wheel_radius_right_cm"]) / 2 / rb["nominal_radius_cm"]
                print(f"         wheel scale {r['wheel_scale']} (sim: mean wheel radius / nominal {ws:.4f}, "
                      f"less ~0.2% slip) [{r['wheel_scale_quality']}]")
            if r.get("gyro_scale"):
                print(f"         gyro scale {r['gyro_scale']} (sim {1 / (1 + rb['gyro_scale_err']):.5f}) "
                      f"[{r['gyro_scale_quality']}]")
            if r.get("spin_led_fwd_cm"):
                print(f"         light from turning centre: fwd {r['spin_led_fwd_cm']} left {r['spin_led_left_cm']} cm "
                      f"(sim {led_fwd + m['axle_to_center_cm']:.2f}, 0) [{r['led_offset_quality']}]")
    if errs:
        a = np.abs(errs)
        print(f"CALIB  heading error vs truth.csv: {len(a)} events, mean |err| {a.mean():.2f} deg, max {a.max():.2f} deg")
    if offs:
        o = np.array(offs)
        print(f"CALIB  LED offset: forward median {np.median(o[:, 0]):.2f} cm (sim model {led_fwd:.2f}), "
              f"left median {np.median(o[:, 1]):.2f} cm (model 0); spread (sd) {o[:, 0].std():.2f} / {o[:, 1].std():.2f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
