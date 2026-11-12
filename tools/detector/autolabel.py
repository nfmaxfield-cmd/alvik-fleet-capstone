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
    python3 tools/detector/autolabel.py <run folder> --pictures --out <labels root>
                                                  runs with no tags and no video (home): the event pictures, labelled
                                                  from the camera track + the robots' own checked headings (below)
The whole command for the real footage: docs/how-to/train-detector.md.

Changelog
  2026-11-10 e13  first version (roof-tag reads, saved video, evaluate_stops, --sim)
  2026-11-12 dv1  --pictures: runs without tags or video (the 11 Nov home run); camera model fitted from the pictures,
                  labels checked against a robots-removed empty board, headings kept only where the camera agrees,
                  one recording per time block (held-out stretches of time, not random frames)
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

VERSION = "autolabel-2026-11-12dv1"
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


# ---------------------------------------------------------------------------------------------- runs with no tags
# (detector-v1, 12 Nov 2026) The home runs have no roof tags and no saved video: only the event pictures
# (<run>/pictures.csv + pictures/<HHMMSS>_<robot>_<why>_<cam>.jpg, one per camera per event), the tag bridge's camera
# track (bridge/track.csv: every robot's position each picture, from motion / lights) and each robot's own fused pose
# (follow.csv: x, y, heading, 10 a second). No camera model is saved in the run folder either (the bridge's calibration
# stays on the Pi), so the camera model is DISCOVERED from the pictures themselves (Observe, then trust):
#   1. the empty board: the median of the run's pictures (robots move, the board does not); its outline gives a
#      starting guess of the floor -> picture map (the corner order is tried all ways round and scored);
#   2. the robots' blobs (picture minus empty board) are paired with the camera track at the picture's time and a
#      floor -> picture homography is refitted on them (RANSAC, the pairing radius shrinking each round). It maps a
#      robot's floor position to the centre of its visible body, which is what the detector should learn;
#   3. a "clean" empty board is rebuilt with every labelled robot cut out (a robot parked for an hour would otherwise
#      sit in the median), and each label is checked against it: no robot-coloured change under a label = the label
#      disagrees with the picture, and that picture is dropped (it would teach "nothing here" on a robot or the reverse);
#   4. heading: the robot's own heading (follow.csv), kept only where the camera confirms it: driving forward along it
#      (camera track, within the run's own agreement spread), or standing with that heading unchanged since the last
#      confirmed moment and its own position within the run's own spread of the camera's. Otherwise "heading unknown".
#      (A first version also accepted driving backward and any standing robot whose position agreed: the held-out
#      check found a robot parked for an hour at the edge labelled ~180 deg wrong that way; fixed before training.)
PIC_TRACK_GAP_X = 2.5    # a track position is interpolated only between reads at most this many median read
                         # intervals apart. CHOSEN (not derived); DOE: 1.5 / 2.5 / 4 against the label check's drop rate
PIC_BODY_CM = 9.5        # an Alvik's roof, edge to edge (FRONT_CM above uses the same). Measured, not chosen
PIC_DISC = 0.2          # the label check looks at a disc this many bodies across round the label (the roof's middle).
                         # CHOSEN; DOE: 0.15 / 0.2 / 0.3 against the share of labels a person would call right
PIC_AGREE_PCT = 95       # the heading check's tolerance: this percentile of the run's own travel-vs-own disagreement


def _rows_by_robot(path, tcol, cols, keep=None):
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if keep is not None and not keep(r):
                continue
            try:
                out.setdefault(r["robot"], []).append(tuple([float(r[tcol])] + [float(r[c]) for c in cols]))
            except (KeyError, ValueError):
                continue
    for v in out.values():
        v.sort()
    return out


_TIMES = {}


def _times(rows):
    """The rows' times (cached: the lists are long and read many times)."""
    k = id(rows)
    if k not in _TIMES or len(_TIMES[k][0]) != len(rows) or _TIMES[k][1] is not rows:
        _TIMES[k] = ([r[0] for r in rows], rows)
    return _TIMES[k][0]


def _lin(rows, t, gap):
    """(x, y) linearly between the reads either side of t, or None if they are more than gap apart."""
    ts = _times(rows)
    k = bisect.bisect_left(ts, t)
    if k == 0 or k >= len(rows) or rows[k][0] - rows[k - 1][0] > gap:
        return None
    a, b = rows[k - 1], rows[k]
    f = (t - a[0]) / max(1e-9, b[0] - a[0])
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])


def board_quad(bg):
    """The board's outline in an empty-board picture: 4 corners (px), or None. Only a starting guess."""
    g = cv2.GaussianBlur(bg, (7, 7), 0)
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((25, 25), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(th)
    if n < 2:
        return None, None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    m = cv2.morphologyEx((lab == k).astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    h = cv2.convexHull(max(cs, key=cv2.contourArea))
    peri = cv2.arcLength(h, True)
    for eps in np.linspace(0.005, 0.1, 40):
        ap = cv2.approxPolyDP(h, eps * peri, True)
        if len(ap) <= 4:
            break
    if len(ap) != 4:
        return None, m
    return ap.reshape(-1, 2).astype(np.float32), m


def _blobs(img, bg, mask, thr):
    d = cv2.absdiff(cv2.GaussianBlur(img, (5, 5), 0), cv2.GaussianBlur(bg, (5, 5), 0))
    m = ((d > thr) & (mask > 0)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    return [cen[i] for i in range(1, n) if 150 < st[i, cv2.CC_STAT_AREA] < 4000]


def _proj(H, x, y):
    v = H @ np.array([x, y, 1.0])
    return v[0] / v[2], v[1] / v[2]


def fit_camera(greys, positions, board):
    """Floor cm -> picture px homography from the pictures alone. greys: list of grey pictures; positions: per picture
    {robot: (x, y) or None}; board: (x0, y0, x1, y1) in cm. Returns (H, info) or (None, info)."""
    bg = np.median(np.stack(greys), axis=0).astype(np.uint8)
    q, m = board_quad(bg)
    info = dict(board_quad_px=None if q is None else q.tolist())
    if q is None:
        return None, info
    mask = cv2.dilate(m, np.ones((41, 41), np.uint8))
    noise = float(np.median(np.abs(np.stack(greys[:40]).astype(np.int16) - bg.astype(np.int16))))
    thr = max(20.0, 8.0 * noise + 10.0)           # robot-vs-board change: well above the pictures' own noise
    blobs = [_blobs(g, bg, mask, thr) for g in greys]
    x0, y0, x1, y1 = board
    W = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    best = None
    import itertools
    for perm in itertools.permutations(range(4)):
        H = cv2.getPerspectiveTransform(np.float32([W[i] for i in perm]), q)
        s = 0.0
        for bl, pos in zip(blobs, positions):
            for xy in pos.values():
                if xy is None or not bl:
                    continue
                u = np.array(_proj(H, *xy))
                s += min(40.0, min(float(np.hypot(*(np.array(b) - u))) for b in bl))
        if best is None or s < best[0]:
            best = (s, H)
    H = best[1]
    res = None
    for rad in (40, 25, 15, 10, 8):
        src, dst = [], []
        for bl, pos in zip(blobs, positions):
            for xy in pos.values():
                if xy is None or not bl:
                    continue
                u = np.array(_proj(H, *xy))
                d = sorted((float(np.hypot(*(np.array(b) - u))), k) for k, b in enumerate(bl))
                if d[0][0] < rad and not (len(d) > 1 and d[1][0] < rad):     # unambiguous pairs only
                    src.append(xy)
                    dst.append(bl[d[0][1]])
        if len(src) < 8:
            break
        H2, inl = cv2.findHomography(np.float32(src), np.float32(dst), cv2.RANSAC, 6.0)
        if H2 is None:
            break
        H = H2
        r = np.array([np.hypot(*(np.array(_proj(H, *s_)) - np.array(d_))) for s_, d_ in zip(src, dst)])
        res = dict(radius_px=rad, pairs=len(src), inliers=int(inl.sum()), median_px=round(float(np.median(r)), 2),
                   p90_px=round(float(np.percentile(r, 90)), 2))
    info.update(fit=res, change_threshold=round(thr, 1))
    return H, info


def label_pictures_run(run, out_root, blocks=8, board=None, log=print):
    """Labels from a run's event pictures (no tags, no video): one labels folder per camera and TIME BLOCK
    (<run>_<cam>_t<k>, blocks of equal picture count in time order), so train.py's split by recording holds out whole
    stretches of time. Writes floor_calibration.json (the discovered homography) into each recording, so evaluate.py
    reports cm. Returns {recording: stats}."""
    name = os.path.basename(os.path.normpath(run))
    pics = []
    with open(os.path.join(run, "pictures.csv"), newline="") as f:
        for r in csv.DictReader(f):
            pics.append((float(r["t"]), r["files"].split(";")))
    pics.sort()
    track = _rows_by_robot(os.path.join(run, "bridge", "track.csv"), "t", ["x", "y"],
                           keep=lambda r: r.get("status") == "seen")
    own = _rows_by_robot(os.path.join(run, "follow.csv"), "t_recv", ["x_cm", "y_cm", "th_deg"],
                         keep=lambda r: r.get("state") not in ("nopose", ""))
    if board is None:
        try:
            with open(os.path.join(run, "map.json")) as f:
                fl = json.load(f)["floor"]
            board = (fl["x_min"], fl["y_min"], fl["x_max"], fl["y_max"])
        except (OSError, KeyError, ValueError):
            board = (0.0, 0.0, 122.0, 122.0)
    dts = [np.median(np.diff([r[0] for r in v])) for v in track.values() if len(v) > 2]
    gap = PIC_TRACK_GAP_X * float(np.median(dts)) if dts else 1.5
    robots = sorted(track)
    pos = [{rb: _lin(track[rb], t, gap) for rb in robots} for t, _ in pics]

    # heading check, part 1: the run's own agreement spreads (travel vs own heading; own vs camera position)
    def own_at(rb, t):
        v = own.get(rb) or []
        ts = _times(v)
        k = bisect.bisect_left(ts, t)
        c = [j for j in (k - 1, k) if 0 <= j < len(v) and abs(v[j][0] - t) <= 0.5]
        return v[min(c, key=lambda j: abs(v[j][0] - t))] if c else None

    def travel(rb, t):
        a, b = _lin(track[rb], t - 1.0, gap), _lin(track[rb], t + 1.0, gap)
        if a is None or b is None or math.hypot(b[0] - a[0], b[1] - a[1]) < 4.0:    # 4 cm in 2 s: moving (chosen)
            return None
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

    agree, offs = [], []
    for rb in robots:
        for t, x, y in track[rb][::20]:
            o, tv = own_at(rb, t), travel(rb, t)
            if o is None:
                continue
            offs.append(math.hypot(o[1] - x, o[2] - y))
            if tv is not None:
                e = abs((o[3] - tv + 180) % 360 - 180)
                if e < 90.0:                            # driving forward (backing up, or a wrong heading: not used)
                    agree.append(e)
    tol_deg = float(np.percentile(agree, PIC_AGREE_PCT)) if agree else 30.0
    tol_cm = float(np.percentile(offs, 75)) if offs else 5.0

    # each robot's moments where the camera CONFIRMED its own heading: driving forward along it (fine steps)
    verified = {}
    for rb in robots:
        vv = []
        for t, x, y in track[rb][::2]:          # every other camera read (about 1 a second)
            o, tv = own_at(rb, t), travel(rb, t)
            if o is not None and tv is not None and abs((o[3] - tv + 180) % 360 - 180) <= tol_deg:
                vv.append((t, o[3]))
        verified[rb] = vv

    def heading_checked(rb, t, xy):
        """The robot's own heading at t, if the camera confirms it: driving forward along it now, or standing with
        its heading unchanged (within the tolerance) since the last moment it was confirmed and its own position
        agreeing with the camera's. Driving backward, or a heading that moved while standing (a re-seed, a turn on
        the spot nobody watched), is left unknown."""
        o, tv = own_at(rb, t), travel(rb, t)
        if o is None:
            return None
        if tv is not None:
            return o[3] if abs((o[3] - tv + 180) % 360 - 180) <= tol_deg else None
        if math.hypot(o[1] - xy[0], o[2] - xy[1]) > tol_cm:
            return None
        vv = verified.get(rb) or []
        k = bisect.bisect_right(_times(vv), t) - 1
        if k < 0 or abs((o[3] - vv[k][1] + 180) % 360 - 180) > tol_deg:
            return None
        return o[3]

    res = {}
    cams = sorted({fn.rsplit("_", 1)[1][:-4] for _, fs in pics for fn in fs})
    for cam in cams:
        files = [next((fn for fn in fs if fn.endswith("_%s.jpg" % cam)), None) for _, fs in pics]
        idx = [k for k, fn in enumerate(files) if fn and os.path.exists(os.path.join(run, "pictures", fn))]
        imgs = {k: cv2.imread(os.path.join(run, "pictures", files[k])) for k in idx}
        greys = {k: cv2.cvtColor(imgs[k], cv2.COLOR_BGR2GRAY) for k in idx}
        H, info = fit_camera([greys[k] for k in idx], [pos[k] for k in idx], board)
        if H is None:
            log("%s %s: no camera model could be found from the pictures; skipped" % (name, cam))
            continue
        hgt, wid = greys[idx[0]].shape

        def body_px(x, y):
            a, b = np.array(_proj(H, x, y)), np.array(_proj(H, x + PIC_BODY_CM, y))
            c = np.array(_proj(H, x, y + PIC_BODY_CM))
            return 0.5 * (np.hypot(*(b - a)) + np.hypot(*(c - a)))

        # the clean empty board: each picture's labelled robots cut out, then the median of what is left
        stack = np.full((len(idx), hgt, wid), np.nan, np.float32)
        for j, k in enumerate(idx):
            g = greys[k].astype(np.float32)
            for xy in pos[k].values():
                if xy is not None:
                    u, v = _proj(H, *xy)
                    cv2.circle(g, (int(round(u)), int(round(v))), int(0.9 * body_px(*xy)), float("nan"), -1)
            stack[j] = g
        clean = np.nanmedian(stack, axis=0)
        clean = np.where(np.isnan(clean), np.nanmedian(stack), clean).astype(np.float32)
        del stack
        # the label check's bar, from the run itself: how much an EMPTY spot of the board changes against the clean
        # board (random spots away from every label), relative to each picture's own noise; a label must stand out
        # above the 99th percentile of those (washed-out robots under the lamp still do; an empty spot does not)
        rng = np.random.default_rng(0)
        diffs = {}

        def change(k, u, v, r_):
            if k not in diffs:
                d = np.abs(cv2.GaussianBlur(greys[k], (3, 3), 0).astype(np.float32) - cv2.GaussianBlur(clean, (3, 3), 0))
                diffs.clear()
                diffs[k] = (d, max(1.0, float(np.median(d))))
            d, noise = diffs[k]
            m = np.zeros((hgt, wid), np.uint8)
            cv2.circle(m, (int(round(u)), int(round(v))), r_, 1, -1)
            return float(d[m > 0].mean()) / noise

        empty = []
        for k in idx:
            here = [_proj(H, *xy) for xy in pos[k].values() if xy is not None]
            for _ in range(3):
                x_, y_ = rng.uniform(board[0], board[2]), rng.uniform(board[1], board[3])
                u, v = _proj(H, x_, y_)
                bp = body_px(x_, y_)
                if all(math.hypot(u - a, v - b) > 2.5 * bp for a, b in here) and 0 <= u < wid and 0 <= v < hgt:
                    empty.append(change(k, u, v, max(2, int(PIC_DISC * bp))))
        bar = float(np.percentile(empty, 99)) if len(empty) >= 20 else 4.0
        info["label_bar"] = dict(empty_spots=len(empty), p99=round(bar, 2), median=round(float(np.median(empty)), 2)
                                 if empty else None)
        shares, rows_by_k, dropped, noheading = [], {}, [], 0
        for k in idx:
            t = pics[k][0]
            objs, bad, why = [], False, ""
            for rb in robots:
                xy = pos[k][rb]
                if xy is None:
                    bad, why = True, "no camera position for %s" % rb    # a robot somewhere, unlabelled
                    break
                u, v = _proj(H, *xy)
                if not (0 <= u < wid and 0 <= v < hgt):
                    continue
                share = change(k, u, v, max(2, int(PIC_DISC * body_px(*xy))))
                shares.append(share)
                if share <= bar:
                    bad, why = True, "%s: no robot under its label (change %.1f x noise, bar %.1f)" % (rb, share, bar)
                    break
                th = heading_checked(rb, t, xy)
                hd = None
                if th is not None:
                    hr = math.radians(th)
                    u2, v2 = _proj(H, xy[0] + FRONT_CM * math.cos(hr), xy[1] + FRONT_CM * math.sin(hr))
                    hd = math.degrees(math.atan2(-(v2 - v), u2 - u)) % 360.0
                if hd is None:
                    noheading += 1
                objs.append((rb, u, v, hd))
            if bad:
                dropped.append((k, why))
                continue
            rows_by_k[k] = objs
        n_blk = max(1, int(blocks))
        order = sorted(idx, key=lambda k: pics[k][0])
        for j, k in enumerate(order):
            blk = 1 + min(n_blk - 1, j * n_blk // len(order))
            rec = os.path.join(out_root, "%s_%s_t%d" % (name, cam, blk))
            st = res.setdefault(os.path.basename(rec), dict(folder=rec, frames=0, labelled=0, dropped=0, objects=0,
                                                             heading_unknown=0, rows=[], t0=pics[k][0]))
            st["frames"] += 1
            st["t1"] = pics[k][0]
            if k not in rows_by_k:
                st["dropped"] += 1
                continue
            os.makedirs(os.path.join(rec, "labels", "frames"), exist_ok=True)
            fno = st["labelled"]
            cv2.imwrite(os.path.join(rec, "labels", "frames", "%06d.jpg" % fno), imgs[k], [cv2.IMWRITE_JPEG_QUALITY, 95])
            for rb, u, v, hd in rows_by_k[k]:
                st["rows"].append([fno, "%.3f" % pics[k][0], rb, "alvik", "%.2f" % u, "%.2f" % v,
                                   "" if hd is None else "%.1f" % hd, "0" if hd is None else "1",
                                   "autolabel:camera-track+own-heading", "confirmed", "1.0"])
                st["objects"] += 1
                st["heading_unknown"] += hd is None
            if not rows_by_k[k]:
                st["rows"].append([fno, "%.3f" % pics[k][0], "", "none", "", "", "", "0", "autolabel:camera-track",
                                   "confirmed", ""])
            st["labelled"] += 1
        for rn, st in res.items():
            if not rn.startswith("%s_%s_" % (name, cam)) or "rows" not in st:
                continue
            rec = st["folder"]
            os.makedirs(os.path.join(rec, "labels"), exist_ok=True)
            with open(os.path.join(rec, "labels", "labels.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known", "source",
                            "status", "score"])
                w.writerows(st.pop("rows"))
            with open(os.path.join(rec, "floor_calibration.json"), "w") as f:
                json.dump({"H_floor_to_image": H.tolist(), "source": "fitted from the run's own pictures and camera "
                           "track (autolabel.fit_camera); maps a robot's floor position to its visible body's centre",
                           "fit": info}, f, indent=1)
            meta = dict({k_: v_ for k_, v_ in st.items() if k_ != "folder"}, version=VERSION, run=os.path.abspath(run),
                        cam=cam, camera_fit=info, heading_tol_deg=round(tol_deg, 1), own_vs_camera_tol_cm=round(tol_cm, 2),
                        label_disc=PIC_DISC, track_gap_s=round(gap, 2))
            with open(os.path.join(rec, "autolabel.json"), "w") as f:
                json.dump(meta, f, indent=1)
        log("%s %s: %d pictures, %d dropped (label disagrees with the picture), %d robots without a checked heading; "
            "camera fit %s; label check: bar %s, labels median %.1f x noise" % (
                name, cam, len(idx), len(dropped), noheading, info.get("fit"), info["label_bar"],
                float(np.median(shares or [0]))))
        for k, why in dropped:
            log("   dropped %s: %s" % (files[k], why))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run folders (each with frames, bridge/tag_poses.csv, tagcal/)")
    ap.add_argument("--out", required=True, help="where the labels folders go (one per run and camera)")
    ap.add_argument("--sim", type=int, default=0, help="make this many simulated runs first and label them")
    ap.add_argument("--pictures", action="store_true",
                    help="runs with no tags and no video: label the event pictures (pictures.csv) from the camera "
                         "track and the robots' own headings, camera model fitted from the pictures (detector-v1)")
    ap.add_argument("--blocks", type=int, default=8, help="--pictures: time blocks per camera (one recording each)")
    a = ap.parse_args(argv)
    if a.pictures:
        tot = {}
        for r in a.runs:
            tot.update(label_pictures_run(r, os.path.join(a.out, "labels"), blocks=a.blocks))
        with open(os.path.join(a.out, "autolabel_summary.json"), "w") as f:
            json.dump(tot, f, indent=1)
        print("Labels for %d recording(s) in %s" % (len(tot), os.path.join(a.out, "labels")))
        return 0 if tot else 1
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
