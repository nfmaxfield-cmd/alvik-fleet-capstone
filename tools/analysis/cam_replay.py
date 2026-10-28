#!/usr/bin/env python3
"""Replay a DOE loop's saved tracking-rate frames (both cameras) through the two-camera tracker, offline (2026-10-28).

Why: the 16:29 DOE loop (28 Oct) saw 26 of 29 bodies with the side camera only, and parked Center (turn check) and
Left (crawl 15.0 cm for 10). This replays the frames the loop kept (frames/<camera>/track/<HHMMSS>_<what>.avi + .csv)
with the same tracker the ladder uses (e02_twocam.TwoCamTracker, overhead 12 / side 10 a second, side lag from its
calibration), and writes every camera's OWN sighting of the moving robot picture by picture, so each camera can be
judged alone and against the other.

    python3 -I tools/analysis/cam_replay.py <frames dir> <logs dir> <overhead calib> <side calib> <out.csv>
        [--only 164005_crawl-Left] [--detect plain|norm]

Seeds: the moving robot's spot at the segment's start comes from the robot's own pose in the loop's leg.csv / fix.csv
(its pose is in the camera's floor frame after every camera fix), the last row at or before the segment's first
picture. The other robots are not tracked (they stand still: the floor memory holds them).
"""
import argparse
import csv
import glob
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "experiment"), TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import e02_camera as CAM  # noqa: E402
import e02_twocam as TW  # noqa: E402

RECT = (0.0, 0.0, CAM.BOARD_W_CM, CAM.BOARD_L_CM)


def read_seg(folder, name):
    """[(t, grey img)] of one camera's segment (t: the Pi's time minus 0.04 s, as the ladder's Eye uses)."""
    rows = list(csv.DictReader(open(os.path.join(folder, name + ".csv"))))
    cap = cv2.VideoCapture(os.path.join(folder, name + ".avi"))
    out = []
    for r in rows:
        ok, im = cap.read()
        if not ok:
            break
        out.append((float(r["t"]) - 0.04, cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)))
    return out


def poses(logs):
    """{robot: [(t, x, y)]} from every leg.csv and fix.csv under logs (the robot's own pose; fixes: the camera's)."""
    out = {}
    for p in glob.glob(os.path.join(logs, "**", "leg.csv"), recursive=True):
        for r in csv.DictReader(open(p)):
            try:
                if float(r["x_cm"]) == 0.0 and float(r["y_cm"]) == 0.0:
                    continue                          # (no pose yet: before its first camera fix)
                out.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"])))
            except (ValueError, KeyError):
                pass
    for p in glob.glob(os.path.join(logs, "**", "fix.csv"), recursive=True):
        for r in csv.DictReader(open(p)):
            try:
                out.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["after_x"]), float(r["after_y"])))
            except (ValueError, KeyError):
                pass
    for v in out.values():
        v.sort()
    return out


def seed(ps, robot, t):
    rows = [q for q in ps.get(robot, []) if q[0] <= t + 0.5]
    return rows[-1][1:] if rows else None


def legs(logs, robot, t0, t1):
    """[(t_start, t_done, kind, value, x0, y0, x1, y1)] of the robot's idle legs in [t0, t1] (its own wheels)."""
    out, open_ = [], {}
    for p in glob.glob(os.path.join(logs, "**", "leg.csv"), recursive=True):
        for r in csv.DictReader(open(p)):
            if r["robot"] != robot:
                continue
            t = float(r["t_recv"])
            if not (t0 - 1 <= t <= t1 + 1):
                continue
            key = (r["kind"], r["value"], r["ms"] if r["result"] == "started" else None)
            if r["result"] == "started":
                open_[(r["kind"], r["value"])] = (t, float(r["x_cm"]), float(r["y_cm"]))
            elif r["result"] == "done" and (r["kind"], r["value"]) in open_:
                ts, x0, y0 = open_.pop((r["kind"], r["value"]))
                out.append((ts, t, r["kind"], float(r["value"]), x0, y0, float(r["x_cm"]), float(r["y_cm"])))
    out.sort()
    return out


FIELDS = ["seg", "robot", "t", "cam", "x", "y", "area", "status", "src", "contrast", "known", "thr", "board"]


def measure(v, x, y):
    """Robot-vs-board contrast where the tracker has the robot: the median |picture - floor memory| (exposure
    offset taken out) inside 0.45 of a robot width round its spot; the share of that floor that was SEEN (not filled
    in); the camera's threshold now; the board's median grey level (floor memory)."""
    if v is None or v.frame is None or v.bg is None:
        return dict(contrast="", known="", thr="", board="")
    m = v._disk_mask(v.bg.shape, [(x, y)], 0.45) > 0
    if not m.any():
        return dict(contrast="", known="", thr="", board="")
    d = np.abs(v.frame[m] - v.offset - v.bg[m])
    if getattr(v, "_board_level", None) is None:
        cut = v._cut_mask(v.bg.shape) if v.board_rect is not None else None
        v._board_level = float(np.median(v.bg[cut > 0])) if cut is not None and cut.any() else float(np.median(v.bg))
    return dict(contrast=round(float(np.median(d)), 1), known=round(float((v.known[m] > 0).mean()), 2),
                thr=round(float(v.thr), 1), board=round(v._board_level, 1))


def drive_seeds(logs):
    """{drive tag: (x, y)} from each drive folder's map.json (where the camera had the robot when it started)."""
    import json
    out = {}
    for p in glob.glob(os.path.join(logs, "**", "map.json"), recursive=True):
        try:
            m = json.load(open(p))
            out[os.path.basename(os.path.dirname(p)).replace(".", "-")] = tuple(m["robots"][0]["start"][:2])
        except (OSError, ValueError, KeyError, IndexError):
            pass
    return out


def replay(frames, logs, ocal, scal, only=None, detect=None, log=print, seeds=None):
    """Rows: segment, robot, t, cam (overhead | side | fused), x, y, area, status, src."""
    ps = poses(logs)
    cam = CAM.load_camera(ocal)
    scam, lag, bias = TW.load_side(scal)
    rows = []
    for ov in sorted(glob.glob(os.path.join(frames, "overhead", "track", "*.avi"))):
        name = os.path.basename(ov)[:-4]
        if only and not any(o in name for o in only):
            continue
        robot = name.split("_", 1)[1].split("-")[1] if name.split("_", 1)[1].startswith(("drive",)) else \
            name.split("-")[-1]
        sd = os.path.join(frames, "side", "track")
        if not os.path.exists(os.path.join(sd, name + ".avi")):
            continue
        o, s = read_seg(os.path.dirname(ov), name), read_seg(sd, name)
        if len(o) < 8 or len(s) < 8:
            continue
        ds = drive_seeds(logs)
        tag = name.split("_", 1)[1]
        sp = (seeds or {}).get(name) or ds.get(tag) or seed(ps, robot, o[0][0])
        if sp is None:
            log("  %s: no seed for %s" % (name, robot))
            continue
        trk = TW.TwoCamTracker(cam, board_center=(61.0, 61.0), board_rect=RECT, margin_cm=8.0, side_cam=scam,
                               side_lag_s=lag, side_bias=bias)
        if detect:
            detect(trk)
        trk.set_background([im for _, im in o[:5]], robots=[sp])
        trk.set_side_background([im for _, im in s[:5]], robots=[sp])
        trk.add(robot, *sp, t=o[0][0])
        ev = sorted([(t, "overhead", im) for t, im in o] + [(t, "side", im) for t, im in s], key=lambda q: q[0])
        for t, c, im in ev:
            if c == "overhead":
                trk.push(im, t)
            else:
                trk.side_t = None
                trk.push_side(im, t)
            tr = trk.tracks[robot]
            ob = tr.obs.get(c)
            meas = measure(trk if c == "overhead" else trk.side, tr.x, tr.y)
            if ob is not None and abs(ob[0] - (t - (trk.side_lag if c == "side" else 0.0))) < 1e-6:
                if ob[3] == "seen":
                    x, y = ob[1], ob[2]
                    if c == "side":
                        x, y = x + trk.bias[0], y + trk.bias[1]     # the side's own spot (no learned offset)
                    rows.append(dict(seg=name, robot=robot, t=round(ob[0], 3), cam=c, x=round(x, 2), y=round(y, 2),
                                     area=ob[5] if len(ob) > 5 else "", status="seen", src="", **meas))
                else:
                    rows.append(dict(seg=name, robot=robot, t=round(ob[0], 3), cam=c, x="", y="", area="",
                                     status=ob[3], src="", **meas))
            rows.append(dict(seg=name, robot=robot, t=round(t, 3), cam="fused", x=round(tr.x, 2), y=round(tr.y, 2),
                             area="", status=tr.status, src=tr.src or ""))
        log("  %s %s seed (%.1f, %.1f): %s" % (name, robot, sp[0], sp[1], trk.counts.get(robot)))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames")
    ap.add_argument("logs")
    ap.add_argument("ocal")
    ap.add_argument("scal")
    ap.add_argument("out")
    ap.add_argument("--only", action="append", default=[])
    ap.add_argument("--seed", action="append", default=[], help="SEGMENT=x,y (where the screen said it stood)")
    ap.add_argument("--hyst", type=float, default=None, help="outlines grown down to this share of the threshold over filled-in floor (0: off; default: "
                         "the tracker's own, e02_twocam.HYST_LOW)")
    ap.add_argument("--light-seed", action="store_true")
    ap.add_argument("--thr-min", type=float, default=None, help="the detector's least threshold (16 as built)")
    a = ap.parse_args()

    def detect(trk):
        for v in (trk, trk.side):
            if v is None:
                continue
            v.HYST_LOW = None if a.hyst is not None and a.hyst <= 0 else (a.hyst if a.hyst is not None else v.HYST_LOW)
            v.LIGHT_SEED = a.light_seed
            if a.thr_min is not None:
                v.thr_min = v.thr = a.thr_min
    seeds = {k: tuple(float(v) for v in xy.split(",")) for k, xy in (q.split("=") for q in a.seed)}
    rows = replay(a.frames, a.logs, a.ocal, a.scal, a.only or None, detect=detect, seeds=seeds)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, restval="")
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
