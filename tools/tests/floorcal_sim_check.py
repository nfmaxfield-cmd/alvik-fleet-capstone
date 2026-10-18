#!/usr/bin/env python3
"""Check tools/calibrate_floor.py against a simulator session's true camera.

    python tools/sim/simulate.py --out sim_out --video --seed 3 --max-runs 2
    python tools/tests/floorcal_sim_check.py sim_out                      # click modes (simulated clicks)
    python tools/tests/floorcal_sim_check.py sim_out --survey --video-offset 0.15   # + survey mode

Click modes: clicks are simulated from the true camera with 1 px noise:
  axes      the taped axes' 10 cm marks (as Nick would click them), camera height given
  axes,no h the same without the camera height (robot tops then off by ~3.5%)
  axes+dots the axes plus 5 pencil dots measured from both axes
  robot     10 E00-style robot placements (robot top, 4 cm up), tape noise 0.2 cm,
            with the camera height
Survey: the runs' own odometry + the simulator's pretend detector, tape start
poses only (as after moving the camera).

Reported: floor error (cm) of the fitted calibration against the true camera
over the E01 area (x -10..130, y -40..60) and over everything the camera sees,
for floor points (z 0) and the robot's top (z 4).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import calibrate_floor as cf      # noqa: E402
from conftest import floor_error   # noqa: E402


class TrueCam:
    """The simulator's camera, from its floor_calibration.json."""

    def __init__(self, path):
        t = json.load(open(path))
        c = t["camera"]
        self.R = np.array(c["R"])
        self.X0, self.Y0 = c["center_floor_cm"]
        self.Hc, self.f, self.k1 = c["height_cm"], t["f_px"], t["k1"]
        self.cx, self.cy = t["cx"], t["cy"]
        self.W, self.H = t["image_size"]

    def project(self, x, y, z=0.0, ideal=False):
        p = self.R @ np.array([x - self.X0, -(y - self.Y0), self.Hc - z])
        xn, yn = p[0] / p[2], p[1] / p[2]
        fac = 1 if ideal else 1 + self.k1 * (xn * xn + yn * yn)
        return self.f * xn * fac + self.cx, self.f * yn * fac + self.cy


def errors(cal, cam, label, plane=False):
    out = {}
    for name, area in (("E01 area", (-10, -40, 130, 60)), ("whole view", (-300, -300, 400, 400))):
        for z in ((4.0,) if plane else (0.0, 4.0)):
            e = floor_error(cal, cam, area, z=0.0 if plane else z)
            out[f"{name} z={z:g}"] = (float(e.mean()), float(np.percentile(e, 95)), float(e.max()))
            print(f"  {label:10} {name:10} z={z:g}: mean {e.mean():.2f}  p95 {np.percentile(e, 95):.2f}  "
                  f"max {e.max():.2f} cm  ({len(e)} points)")
    return out


def click_points(cam, pts, z=0.0, px_noise=1.0, tape_noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for i, (x, y) in enumerate(pts):
        u, v = cam.project(x, y, z)
        out.append(dict(id=i + 1, kind="robot" if z else "floor", u=u + rng.normal(0, px_noise),
                        v=v + rng.normal(0, px_noise), x=x + rng.normal(0, tape_noise),
                        y=y + rng.normal(0, tape_noise), z=z, enabled=True))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sim")
    ap.add_argument("--survey", action="store_true")
    ap.add_argument("--video-offset", type=float, help="for --survey (else found from the flashes: slow)")
    ap.add_argument("--height-cm", type=float, help="survey: camera height given (default: the true one)")
    args = ap.parse_args(argv)
    cam = TrueCam(os.path.join(args.sim, "floor_calibration.json"))
    size = (cam.W, cam.H)
    axes = [(x, 0.0) for x in range(-10, 141, 10)] + [(0.0, y) for y in range(-40, 61, 10) if y]
    dots = [(50, 40), (100, 40), (120, -30), (40, -30), (100, -20)]
    spots = [(15, 30), (45, -25), (70, 45), (100, 10), (120, -30), (20, -35), (60, 0), (90, 50), (125, 40), (5, 55)]
    results = {}
    print(f"True camera: height {cam.Hc} cm, k1 {cam.k1}")
    for label, pts, kw in (("axes", click_points(cam, axes), dict(height_cm=cam.Hc)),
                           ("axes,no h", click_points(cam, axes), {}),
                           ("axes+dots", click_points(cam, axes + dots), dict(height_cm=cam.Hc)),
                           ("robot", click_points(cam, spots, z=4.0, tape_noise=0.2), dict(height_cm=cam.Hc))):
        res = cf.fit(pts, size, k1_mode="auto", **kw)
        print(f"{label}: fit RMS {res['rms_cm']:.2f} cm, k1 {res['k1']:.3f}, {len(res['warnings'])} warning(s)")
        results[label] = errors(cf.to_calibration(res), cam, label)
    if args.survey:
        import floorcal_runs as fr
        runs, fits = fr.rn.load_runs(os.path.join(args.sim, "session"), os.path.join(args.sim, "measurements.csv"))
        off = args.video_offset
        if off is None:
            off = fr.rn.video_offset(os.path.join(args.sim, "session"), os.path.join(args.sim, "recording"), fits)[0]
        dets = fr.read_detections(os.path.join(args.sim, "camera_detections.csv"))
        pairs = fr.survey_pairs(runs, dets, off, fr.odometry_paths(runs))
        res = fr.fit_survey(pairs, size, height_cm=args.height_cm or cam.Hc, k1_mode="auto")
        print(f"survey: {res['n_inliers']}/{res['n_points']} pairs, path RMS {res['rms_cm']:.2f} cm, k1 {res['k1']:.3f}, "
              f"clock {list(res['survey']['sessions'].values())} ms")
        results["survey"] = errors(cf.to_calibration(res), cam, "survey")
    return results


if __name__ == "__main__":
    main()
