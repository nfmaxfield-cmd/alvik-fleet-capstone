#!/usr/bin/env python3
"""Labelled training frames made automatically from a run's own roof-tag reads (build e13, 10 Nov 2026).

Where a roof tag was read, the robot's centre and heading are known (the tag sits over the turning point: tag offset
0 cm, tag_geom). So a run that saved its pictures is a labelled recording for the camera detector (CentreNet,
tools/detector), with no person clicking:

  1. FRAMES   the run's pictures: frames_where.txt points to them (the USB stick), else <run>/frames; each camera's
              tracking parts track/<HHMMSS>_<label>.avi with .csv (i, t = Pi clock, pi_frame) (doe_loop.CamFrames).
  2. TRUTH    bridge/tag_poses.csv (every tag read: t, cam, robot, x, y, heading, sigma_cm, used); only CONFIDENT reads
              are kept: used == yes, sigma_cm at most the camera's 1-sigma (safety_rules.CAM_SIGMA_CM, 1 cm), and the
              frame's time within half a picture interval of a read on each side (interpolated between the two).
  3. PROJECT  each robot's centre (x, y at the roof's height) and a front point (a third of a body ahead) through the
              site's camera model (<run>/tagcal/<cam>.json, else the site's) into the picture: x_px, y_px and the
              heading in the labels' convention (degrees counter-clockwise from picture-right, picture-up positive).
  4. SAFE     a frame is written only if EVERY robot the run knew was where its track (run/camera_track.csv) puts it
              is labelled: a robot in the picture but without a confident read would teach "nothing here". Frames
              with a robot unlabelled are left out (counted).
  5. WRITE    <out>/<run>_<cam>/labels/labels.csv + frames/<frame:06d>.jpg, the labels format of docs/data-formats.md
              (status "confirmed", source "autolabel:tag"); one recording per run and camera, so train.py's split by
              recording is a split by run.

Then evaluate_stops(): a detector (anything with load()/predict(), docs/data-formats.md) against the stops of held-out
runs, whose truth is the camera's median during each stop: detection rate, false detections, centre error in cm (the
detection projected back onto the table) and heading error.

    python3 tools/detector/autolabel.py <run folder> [<run folder> ...] --out <labels root>
    python3 tools/detector/autolabel.py --sim --out <dir>        a simulated run (pictures, tag reads, stops) end to end
The whole command for the real footage: docs/how-to/train-detector.md.
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "sim"),
           os.path.join(TOOLS, "label"), os.path.join(TOOLS, "experiment")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import cv2  # noqa: E402

VERSION = "autolabel-2026-11-10e13"
CONF_SIGMA_CM = 1.0      # a read is confident at or under the camera's 1-sigma (safety_rules.CAM_SIGMA_CM). Not chosen
FRONT_CM = 9.5 / 3.0     # the front point: a third of a body ahead (the detector's own front-point convention, README)
ROOF_Z_CM = 6.0          # the roof's height when the run does not say (the school tags' 6.2 cm typed). Not chosen here


# ---------------------------------------------------------------------------------------------- reading a run
def frames_dir(run):
    p = os.path.join(run, "frames_where.txt")
    if os.path.exists(p):
        with open(p) as f:
            d = f.read().strip().splitlines()[0].strip()
        if os.path.isdir(d):
            return d
    return os.path.join(run, "frames")


def tag_reads(run):
    """{(cam, robot): sorted [(t, x, y, heading)]} of the confident reads."""
    p = os.path.join(run, "bridge", "tag_poses.csv")
    out = {}
    if not os.path.exists(p):
        return out
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            try:
                if (r.get("used") or "yes") != "yes" or float(r.get("sigma_cm") or 99) > CONF_SIGMA_CM:
                    continue
                out.setdefault((r["cam"], r["robot"]), []).append(
                    (float(r["t"]), float(r["x"]), float(r["y"]), float(r["heading"])))
            except (KeyError, ValueError):
                continue
    for v in out.values():
        v.sort()
    return out


def tracks(run):
    """{robot: sorted [(t, x, y)]} from run/camera_track.csv (any source): where every robot was."""
    p = os.path.join(run, "run", "camera_track.csv")
    out = {}
    if os.path.exists(p):
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    out.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
                except (KeyError, ValueError):
                    continue
    for v in out.values():
        v.sort()
    return out


def model_for(run, cam):
    from cameras import CameraModel
    for p in (os.path.join(run, "tagcal", cam + ".json"), os.path.join(run, "site", "tagcal", cam + ".json")):
        if os.path.exists(p):
            return CameraModel.load(p)
    return None


def _interp(rows, t, max_gap):
    """(x, y, heading) at t between the reads on each side (each within max_gap of t), or None."""
    ts = [r[0] for r in rows]
    k = bisect.bisect_left(ts, t)
    if k == 0 or k >= len(rows):
        if k < len(rows) and abs(rows[k][0] - t) <= max_gap * 0.25:
            return rows[k][1:]
        if k > 0 and abs(rows[k - 1][0] - t) <= max_gap * 0.25:
            return rows[k - 1][1:]
        return None
    a, b = rows[k - 1], rows[k]
    if t - a[0] > max_gap or b[0] - t > max_gap:
        return None
    f = (t - a[0]) / max(1e-9, b[0] - a[0])
    dh = (b[3] - a[3] + 180.0) % 360.0 - 180.0
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dh


def _near(rows, t, max_gap):
    ts = [r[0] for r in rows]
    k = bisect.bisect_left(ts, t)
    best = None
    for j in (k - 1, k):
        if 0 <= j < len(rows) and abs(rows[j][0] - t) <= max_gap and (best is None or abs(rows[j][0] - t) <
                                                                       abs(best[0] - t)):
            best = rows[j]
    return best


def project(model, x, y, h_deg, z):
    """(u, v, heading in the labels' convention) or None when outside the picture."""
    u, v = model.project((x, y, z))
    if not (np.isfinite(u) and np.isfinite(v)) or not model.in_image(u, v):
        return None
    hr = math.radians(h_deg)
    u2, v2 = model.project((x + FRONT_CM * math.cos(hr), y + FRONT_CM * math.sin(hr), z))
    return float(u), float(v), math.degrees(math.atan2(-(v2 - v), u2 - u)) % 360.0


# ---------------------------------------------------------------------------------------------- 1-5
def label_run(run, out_root, z=None, log=print):
    """Writes one labels folder per camera of this run. Returns {cam: dict(frames, labelled, skipped_unlabelled,
    skipped_no_truth, folder)}."""
    name = os.path.basename(os.path.normpath(run))
    reads = tag_reads(run)
    trk = tracks(run)
    z = ROOF_Z_CM if z is None else float(z)
    try:
        with open(os.path.join(run, "tags.json")) as f:
            z = float(json.load(f).get("height_cm") or z)
    except (OSError, ValueError, TypeError):
        pass
    fd = frames_dir(run)
    res = {}
    for cam_dir in sorted(glob.glob(os.path.join(fd, "*"))):
        cam = os.path.basename(cam_dir)
        parts = sorted(glob.glob(os.path.join(cam_dir, "track", "*.avi")))
        model = model_for(run, cam)
        if not parts or model is None:
            continue
        robots = sorted({r for (c, r) in reads if c == cam} | set(trk))
        rec = os.path.join(out_root, "%s_%s" % (name, cam))
        os.makedirs(os.path.join(rec, "labels", "frames"), exist_ok=True)
        rows, st = [], dict(frames=0, labelled=0, skipped_unlabelled=0, skipped_no_truth=0, folder=rec)
        fno = 0
        for part in parts:
            times = []
            with open(part[:-4] + ".csv", newline="") as f:
                for r in csv.DictReader(f):
                    times.append(float(r["t"]))
            dt_pic = float(np.median(np.diff(times))) if len(times) > 2 else 0.1
            gap = max(dt_pic, 0.05)
            cap = cv2.VideoCapture(part)
            i = 0
            while True:
                ok, img = cap.read()
                if not ok or i >= len(times):
                    break
                t = times[i]
                i += 1
                st["frames"] += 1
                objs, bad = [], False
                for rb in robots:
                    p = _interp(reads.get((cam, rb), []), t, gap)
                    if p is not None:
                        q = project(model, p[0], p[1], p[2], z)
                        if q is not None:
                            objs.append((rb, q))
                        continue
                    w = _near(trk.get(rb, []), t, 1.0)
                    if w is not None and project(model, w[1], w[2], 0.0, z) is not None:
                        bad = True             # in the picture, no confident read: the frame would mislead
                if bad:
                    st["skipped_unlabelled"] += 1
                    continue
                if not objs and not any(trk.values()):
                    st["skipped_no_truth"] += 1
                    continue
                cv2.imwrite(os.path.join(rec, "labels", "frames", "%06d.jpg" % fno), img,
                            [cv2.IMWRITE_JPEG_QUALITY, 90])
                if not objs:
                    rows.append([fno, "%.4f" % t, "", "none", "", "", "", "0", "autolabel:tag", "confirmed", ""])
                for k, (rb, (u, v, hd)) in enumerate(objs):
                    rows.append([fno, "%.4f" % t, rb, "alvik", "%.2f" % u, "%.2f" % v, "%.1f" % hd, "1",
                                 "autolabel:tag", "confirmed", "1.0"])
                st["labelled"] += 1
                fno += 1
            cap.release()
        with open(os.path.join(rec, "labels", "labels.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known", "source",
                        "status", "score"])
            w.writerows(rows)
        with open(os.path.join(rec, "autolabel.json"), "w") as f:
            json.dump(dict(st, version=VERSION, run=os.path.abspath(run), cam=cam, conf_sigma_cm=CONF_SIGMA_CM,
                           roof_z_cm=z), f, indent=1)
        log("%s %s: %d pictures, %d labelled, %d left out (a robot in view without a confident read)" % (
            name, cam, st["frames"], st["labelled"], st["skipped_unlabelled"]))
        res[cam] = st
    return res


# ---------------------------------------------------------------------------------------------- evaluation at stops
def evaluate_stops(run, labels_rec, load, predict, model_path=None, cam="overhead", z=None, settle_s=1.0,
                   radius_px=None):
    """A detector against the stops of a held-out run: during each stop (from settle_s after its start: the 9 Nov
    tours settle within 1 s for 97 % of stops) the robot stands at the camera's median position. Returns dict(
    frames, robots, found, false, rate, centre_cm (median, p95), heading_deg (median, p95), ms_per_frame)."""
    import time
    model = model_for(run, cam)
    z = ROOF_Z_CM if z is None else float(z)
    stops = []
    with open(os.path.join(run, "run", "stops.csv"), newline="") as f:
        for r in csv.DictReader(f):
            try:
                stops.append((float(r["dwell_start_unix"]) + settle_s, float(r["dwell_end_unix"]), r["robot"],
                              float(r["cam_x_cm"]), float(r["cam_y_cm"])))
            except (KeyError, ValueError):
                continue
    labs = {}
    with open(os.path.join(labels_rec, "labels", "labels.csv"), newline="") as f:
        for r in csv.DictReader(f):
            labs.setdefault(int(r["frame"]), []).append(r)
    det = load(model_path)
    errs, herrs, found, false_, n_obj, n_fr, ms = [], [], 0, 0, 0, 0, []
    rad = radius_px
    for fr in sorted(labs):
        t = float(labs[fr][0]["t_wall"])
        here = [s for s in stops if s[0] <= t <= s[1]]
        img = cv2.imread(os.path.join(labels_rec, "labels", "frames", "%06d.jpg" % fr))
        t0 = time.perf_counter()
        preds = [p for p in predict(det, img) if p.get("cls") == "alvik"]
        ms.append((time.perf_counter() - t0) * 1000.0)
        if not here:
            continue
        n_fr += 1
        used = set()
        for s in here:
            n_obj += 1
            q = project(model, s[3], s[4], 0.0, z)
            if q is None:
                continue
            ppc = float(model.px_per_cm(s[3], s[4], z))
            r_ = rad or 0.5 * 9.5 * ppc
            cand = [(math.hypot(p["x"] - q[0], p["y"] - q[1]), k) for k, p in enumerate(preds) if k not in used]
            cand = [c for c in cand if c[0] <= r_]
            if not cand:
                continue
            d, k = min(cand)
            used.add(k)
            found += 1
            p = preds[k]
            xy = model.to_floor(p["x"], p["y"], z)
            errs.append(math.hypot(xy[0] - s[3], xy[1] - s[4]))
            lab = [r for r in labs[fr] if r.get("obj") == s[2] and r.get("heading_deg")]
            if p.get("heading_deg") is not None and lab:
                herrs.append(abs((float(p["heading_deg"]) - float(lab[0]["heading_deg"]) + 180) % 360 - 180))
        false_ += len(preds) - len(used)
    pc = lambda v, q: round(float(np.percentile(v, q)), 2) if v else None  # noqa: E731
    return dict(frames=n_fr, robots=n_obj, found=found, false=false_, rate=round(found / max(1, n_obj), 3),
                centre_cm=(pc(errs, 50), pc(errs, 95)), heading_deg=(pc(herrs, 50), pc(herrs, 95)),
                ms_per_frame_here=pc(ms, 50))


# ---------------------------------------------------------------------------------------------- simulated footage
def make_sim_run(folder, seed=1, n_frames=120, robots=2, plain=False, fps=10.0, t0=1.79e9):
    """A run folder as a real one looks (pictures in frames/overhead/track/*.avi + .csv, bridge/tag_poses.csv from the
    simulated camera's REAL tag detector, run/camera_track.csv, run/stops.csv, tagcal/overhead.json) from
    tools/sim/tag_cam.py: robots that drive, stop 3 s at points, drive on."""
    import tag_cam as TCAM
    import tagcore as TC
    import tag_geom as TG  # noqa: F401  (the bridge's geometry; the centre is the tag's: offset 0)
    import e02_framesaver as FS
    rng = np.random.default_rng(seed)
    cam = TCAM.TagCamera((100.0, 70.0, 150.0), (100.01, 70.0, 0.0), size=(640, 480), hfov_deg=70, name="overhead",
                         table=(0, 0, 200, 140), seed=seed)
    os.makedirs(os.path.join(folder, "tagcal"), exist_ok=True)
    cam.true_model().save(os.path.join(folder, "tagcal", "overhead.json"))
    det = TC.TagDetector("tag25h9")
    names = ["Alpha", "Bravo", "Charlie"][:robots]
    pos = {n: [float(rng.uniform(50, 150)), float(rng.uniform(40, 100)), float(rng.uniform(-180, 180))] for n in names}
    legs = {n: [] for n in names}
    for n in names:
        tt = 0.0
        while tt < n_frames / fps:
            go = float(rng.uniform(1.5, 3.0))
            legs[n].append(("go", tt, tt + go, float(rng.uniform(-60, 60))))
            tt += go
            legs[n].append(("stop", tt, tt + 3.0, 0.0))
            tt += 3.0
    jpegs, times = [], []
    poses_rows, track_rows, stop_rows = [], [], []
    v = 8.0
    acc = {n: [] for n in names}
    for i in range(n_frames):
        t = i / fps
        for n in names:
            p = pos[n]
            lg = next(lg for lg in legs[n] if lg[1] <= t < lg[2])
            if lg[0] == "go":
                p[2] += lg[3] / fps / (lg[2] - lg[1])
                nx, ny = p[0] + v / fps * math.cos(math.radians(p[2])), p[1] + v / fps * math.sin(math.radians(p[2]))
                if 25 < nx < 175 and 25 < ny < 115:
                    p[0], p[1] = nx, ny
                else:
                    p[2] += 180.0
            else:
                acc[n].append((t0 + t, p[0], p[1], lg[1], lg[2]))
        rs = [dict(x=pos[n][0], y=pos[n][1], th=math.radians(pos[n][2]), z=6.0, family="tag25h9", id=k, tag_cm=8.0,
                   roof_cm=9.5) for k, n in enumerate(names)]
        img = cam.render(rs, covered=tuple(range(len(names))) if plain else ())
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        jpegs.append(buf.tobytes())
        times.append(t0 + t)
        ids = {d.id: d for d in det.detect(img)}
        for k, n in enumerate(names):
            track_rows.append([t0 + t, n, pos[n][0], pos[n][1]])
            d = ids.get(k)
            if d is None:
                continue
            # the tag read as the bridge would give it: the truth plus the camera's ~0.3 cm noise, its sigma from size
            sig = 0.3 if d.size_px > 14 else 1.5
            poses_rows.append(["%.4f" % (t0 + t), "overhead", n, k, "%.2f" % (pos[n][0] + rng.normal(0, 0.2)),
                               "%.2f" % (pos[n][1] + rng.normal(0, 0.2)), "%.2f" % (pos[n][2] + rng.normal(0, 0.5)),
                               "%.3f" % sig, "1.0", "1.0", "", "", "", "yes", "", ""])
    d_ = os.path.join(folder, "frames", "overhead", "track")
    os.makedirs(d_, exist_ok=True)
    FS.write_mjpeg_avi(os.path.join(d_, "000000_sim.avi"), jpegs, fps, 640, 480)
    with open(os.path.join(d_, "000000_sim.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["i", "t", "pi_frame"])
        for i, t in enumerate(times):
            w.writerow([i, "%.3f" % t, i])
    os.makedirs(os.path.join(folder, "bridge"), exist_ok=True)
    with open(os.path.join(folder, "bridge", "tag_poses.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cam", "robot", "id", "x", "y", "heading", "sigma_cm", "sigma_deg", "size_ratio",
                    "px_per_cm", "u", "v", "used", "mid_x", "mid_y"])
        w.writerows(poses_rows)
    os.makedirs(os.path.join(folder, "run"), exist_ok=True)
    with open(os.path.join(folder, "run", "camera_track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x_cm", "y_cm"])
        w.writerows(track_rows)
    with open(os.path.join(folder, "run", "stops.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "n", "point", "dwell_start_unix", "dwell_end_unix", "cam_x_cm", "cam_y_cm"])
        for n in names:
            seen = {}
            for (t, x, y, a, b) in acc[n]:
                seen.setdefault((a, b), []).append((x, y))
            for k, ((a, b), xy) in enumerate(sorted(seen.items())):
                if len(xy) >= 5:
                    w.writerow([n, k + 1, "S%d" % (k + 1), "%.3f" % (t0 + a), "%.3f" % (t0 + b),
                                "%.2f" % xy[0][0], "%.2f" % xy[0][1]])
    with open(os.path.join(folder, "tags.json"), "w") as f:
        json.dump({"height_cm": 6.0}, f)
    return folder


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run folders (each with frames, bridge/tag_poses.csv, tagcal/)")
    ap.add_argument("--out", required=True, help="where the labels folders go (one per run and camera)")
    ap.add_argument("--sim", type=int, default=0, help="make this many simulated runs first and label them")
    a = ap.parse_args(argv)
    runs = []
    for r in a.runs:                 # a run folder, or a folder of run folders (PowerShell does not expand *)
        if os.path.isdir(os.path.join(r, "bridge")):
            runs.append(r)
        else:
            runs += sorted(os.path.dirname(os.path.dirname(b)) for b in glob.glob(os.path.join(r, "**", "bridge",
                                                                                              "tag_poses.csv"),
                                                                                 recursive=True))
    for k in range(a.sim):
        runs.append(make_sim_run(os.path.join(a.out, "sim_runs", "simrun%02d" % (k + 1)), seed=k + 1))
    tot = {}
    for r in runs:
        for cam, st in label_run(r, os.path.join(a.out, "labels")).items():
            tot[os.path.basename(st["folder"])] = st
    with open(os.path.join(a.out, "autolabel_summary.json"), "w") as f:
        json.dump(tot, f, indent=1)
    print("Labels for %d recording(s) in %s. Train: python tools/detector/train.py --labels %s --out <dir>" % (
        len(tot), os.path.join(a.out, "labels"), os.path.join(a.out, "labels")))
    return 0 if tot else 1


if __name__ == "__main__":
    sys.exit(main())
