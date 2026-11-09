#!/usr/bin/env python3
"""A run's data for Jay-Sun, in clean files (school build, 2026-11-03). Made at the end of every mission_run, or again:

    python3 tools/fleet/run_exports.py <mission folder>            # camera_track.csv + odometry_raw.csv in <run>/
    python3 tools/fleet/run_exports.py <mission folder> --bundle   # and the raw-files zip for Jay-Sun (never in git)
    ./run-fleet-pi.sh jay-bundle [<mission folder>]                 # the same, the newest school run by default

camera_track.csv (Jay-Sun: "the camera position for every frame while driving, not just at the stops"): every moment a
camera saw a robot, the whole run (start-up, wiggle, driving, stands): the tag bridge's every read (bridge/tag_poses.csv,
refused reads left out), the two cameras' reads of one robot within the bridge's PAIR_S fused inverse-variance as the
bridge fuses them. Lights runs: the camera bridge's seen rows (bridge/track.csv). No-camera runs: no file (nothing
saw the robots).

odometry_raw.csv (Jay-Sun: "the wheel+gyro odometry on its own with no camera correction, as separate columns"):
each robot's dead-reckoned track rebuilt offline from its RAW rows (firmware rawlog.h, raw_hz 50 a second, already
logged; no firmware change): distance from the motor board's own odometry step (x_cm, y_cm, th_deg: the wheels)
projected on its own heading, times wheel_scale; heading from the IMU's yaw (yaw_deg) unwrapped, times yaw_sign x
gyro_scale, minus gyro_drift_dps x time: the firmware's pose.h tick() step for step, with NO camera fix ever applied.
Anchored ONCE: at the robot's first own pose of the run (run/follow.csv: its home, set by the first camera fix or, with
no camera, by the start pose given). Time: the robot's clock (ms) on the Pi's clock by the run's clock sync (median
offset per boot). A robot that restarts mid-run starts a new segment anchored on its own pose then (column anchor).
"""
import argparse
import csv
import datetime as dt
import json
import math
import os
import statistics
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

VERSION = "2026-11-09b"   # 11-09b: stops.csv robot_wifi + tag_id, binding.json + frames_where.txt in the bundle. e11: run/stops.csv (Jay-Sun: each stop findable), visit_order.json in the bundle
PAIR_S = 0.07          # as tag_bridge.PAIR_S: two cameras' reads this close in time are one moment

CAM_COLS = ["t", "t_iso", "robot", "x_cm", "y_cm", "heading_deg", "sigma_cm", "sigma_deg", "pm95_cm", "cams", "n_reads",
            "cams_apart_cm", "source"]
ODO_COLS = ["t", "t_iso", "robot", "boot", "ms", "x_odo_cm", "y_odo_cm", "heading_odo_deg", "dist_cm", "wl_deg",
            "wr_deg", "lib_x_cm", "lib_y_cm", "lib_th_deg", "yaw_deg", "gz", "wheel_scale", "gyro_scale",
            "gyro_drift_dps", "yaw_sign", "segment", "anchor"]


def iso(t):
    return dt.datetime.fromtimestamp(float(t)).astimezone().isoformat(timespec="milliseconds")


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _rows(path):
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def _fuse(reads):
    """[(x, y, th or None, sigma_cm, sigma_deg, cam)] -> x, y, th, sigma_cm, sigma_deg (inverse-variance)."""
    w = [1.0 / max(1e-6, r[3]) ** 2 for r in reads]
    W = sum(w)
    x = sum(wi * r[0] for wi, r in zip(w, reads)) / W
    y = sum(wi * r[1] for wi, r in zip(w, reads)) / W
    hs = [(1.0 / max(1e-6, r[4]) ** 2, r[2]) for r in reads if r[2] is not None]
    th = sd = None
    if hs:
        c = sum(wi * math.cos(math.radians(h)) for wi, h in hs)
        s = sum(wi * math.sin(math.radians(h)) for wi, h in hs)
        th = math.degrees(math.atan2(s, c))
        sd = 1.0 / math.sqrt(sum(wi for wi, _ in hs))
    return x, y, th, 1.0 / math.sqrt(W), sd


def camera_track(mission, out_csv=None):
    """<run>/camera_track.csv. Returns (path or None, {robot: rows})."""
    run = os.path.join(mission, "run")
    out_csv = out_csv or os.path.join(run, "camera_track.csv")
    rows = []
    tp = _rows(os.path.join(mission, "bridge", "tag_poses.csv"))
    if tp:
        by = {}
        for r in tp:
            if r.get("used", "yes") != "yes":
                continue
            try:
                by.setdefault(r["robot"], []).append((float(r["t"]), float(r["x"]), float(r["y"]),
                                                      float(r["heading"]) if r.get("heading") not in ("", None) else None,
                                                      float(r.get("sigma_cm") or 1.0), float(r.get("sigma_deg") or 3.0),
                                                      r.get("cam", "")))
            except (KeyError, ValueError):
                continue
        for n, reads in by.items():
            reads.sort()
            i = 0
            while i < len(reads):
                grp, cams = [reads[i]], {reads[i][6]}
                j = i + 1
                while j < len(reads) and reads[j][0] - reads[i][0] <= PAIR_S and reads[j][6] not in cams:
                    grp.append(reads[j])
                    cams.add(reads[j][6])
                    j += 1
                i = j
                x, y, th, sd, sdd = _fuse([(g[1], g[2], g[3], g[4], g[5], g[6]) for g in grp])
                apart = (math.hypot(grp[0][1] - grp[1][1], grp[0][2] - grp[1][2]) if len(grp) > 1 else 0.0)
                t = max(g[0] for g in grp)
                rows.append([t, n, x, y, th, sd, sdd, 1.96 * sd, "+".join(sorted(cams)), len(grp), apart,
                             "roof tag"])
    else:
        for r in _rows(os.path.join(mission, "bridge", "track.csv")):
            if r.get("status") != "seen":
                continue
            try:
                rows.append([float(r["t"]), r["robot"], float(r["x"]), float(r["y"]),
                             float(r["sent_th"]) if r.get("sent_th") else None, 1.0, None, 1.96,
                             r.get("src") or "overhead", 1, 0.0, "robot's lights"])
            except (KeyError, ValueError):
                continue
    if not rows:
        return None, {}
    rows.sort(key=lambda r: (r[0], r[1]))
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    counts = {}
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CAM_COLS)
        for r in rows:
            w.writerow(["%.4f" % r[0], iso(r[0]), r[1], "%.2f" % r[2], "%.2f" % r[3],
                        "" if r[4] is None else "%.2f" % r[4], "%.3f" % r[5], "" if r[6] is None else "%.2f" % r[6],
                        "%.2f" % r[7], r[8], r[9], "%.2f" % r[10], r[11]])
            counts[r[1]] = counts.get(r[1], 0) + 1
    return out_csv, counts


def _offsets(run):
    """{boot: Pi time - robot ms / 1000 (median of the run's clock syncs)}."""
    by = {}
    for r in _rows(os.path.join(run, "clock_sync.csv")):
        try:
            by.setdefault(r["boot"], []).append(float(r["offset_s"]))
        except (KeyError, ValueError):
            pass
    return {b: statistics.median(v) for b, v in by.items() if v}


def _params(run):
    """{robot: [(t_recv, name, value)]} sorted, for wheel_scale / gyro_scale / gyro_drift_dps."""
    out = {}
    for r in _rows(os.path.join(run, "param.csv")):
        if r.get("name") in ("wheel_scale", "gyro_scale", "gyro_drift_dps"):
            try:
                out.setdefault(r["robot"], []).append((float(r["t_recv"]), r["name"], float(r["value"])))
            except (KeyError, ValueError):
                pass
    for v in out.values():
        v.sort()
    return out


def odometry_raw(mission, out_csv=None):
    """<run>/odometry_raw.csv. Returns (path or None, {robot: rows}, {robot: words})."""
    run = os.path.join(mission, "run")
    out_csv = out_csv or os.path.join(run, "odometry_raw.csv")
    raw = _rows(os.path.join(run, "raw.csv"))
    if not raw:
        return None, {}, {}
    offs = _offsets(run)
    pars = _params(run)
    ysign = {}
    for r in _rows(os.path.join(run, "runinfo.csv")):
        try:
            ysign[r["run"]] = float(r.get("yaw_sign") or 1) or 1.0
        except ValueError:
            ysign[r["run"]] = 1.0
    anchors = {}
    for r in _rows(os.path.join(run, "follow.csv")):
        if r.get("state") == "nopose":
            continue
        try:
            anchors.setdefault((r["robot"], r["run"].split("-")[0]), []).append(
                (int(float(r["ms"])), float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"])))
        except (KeyError, ValueError):
            pass
    by = {}
    for r in raw:
        by.setdefault(r["robot"], []).append(r)
    out, words = [], {}
    for n, rs in by.items():
        rs.sort(key=lambda r: (r["run"].split("-")[0], int(float(r["ms"]))))
        prm = pars.get(n, [])
        seg = 0
        state = None
        boot_prev = None
        for r in rs:
            boot = r["run"].split("-")[0]
            try:
                ms = int(float(r["ms"]))
                lx, ly, lth = float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"])
                yaw = float(r["yaw_deg"])
            except (KeyError, ValueError):
                continue
            off = offs.get(boot)
            t = ms / 1000.0 + off if off is not None else float(r["t_recv"])
            cur = {"wheel_scale": 1.0, "gyro_scale": 1.0, "gyro_drift_dps": 0.0}
            for (tp, k, v) in prm:
                if tp <= float(r["t_recv"]) + 1e-6:
                    cur[k] = v
            ys = ysign.get(r["run"], 1.0)
            if boot != boot_prev:
                state = None
                boot_prev = boot
            if state is None:
                anc = [a for a in anchors.get((n, boot), []) if a[0] >= ms - 100]
                if not anc:
                    continue                       # no own pose on this boot yet: nothing to anchor on
                a = anc[0]
                if a[0] > ms:
                    continue                       # wait for the raw row at (or just after) the anchor's moment
                seg += 1
                state = {"x": a[1], "y": a[2], "h": a[3], "lx": lx, "ly": ly, "yaw": yaw, "ms": ms, "dist": 0.0}
                anchor = "own pose at ms %d (%s)" % (a[0], "start" if seg == 1 else "after a restart")
                words.setdefault(n, []).append("segment %d anchored on its own pose (%.1f, %.1f, %.1f deg) at ms %d"
                                               % (seg, a[1], a[2], a[3], a[0]))
            else:
                dx, dy = lx - state["lx"], ly - state["ly"]
                ds = (dx * math.cos(math.radians(lth)) + dy * math.sin(math.radians(lth))) * cur["wheel_scale"]
                dyaw = wrap180(yaw - state["yaw"])
                dh = ys * cur["gyro_scale"] * dyaw - cur["gyro_drift_dps"] * (ms - state["ms"]) / 1000.0
                mid = math.radians(state["h"] + 0.5 * dh)
                state["x"] += ds * math.cos(mid)
                state["y"] += ds * math.sin(mid)
                state["h"] = wrap180(state["h"] + dh)
                state["dist"] += ds
                state.update(lx=lx, ly=ly, yaw=yaw, ms=ms)
            out.append([t, n, boot, ms, state["x"], state["y"], state["h"], state["dist"], r.get("wl_deg"),
                        r.get("wr_deg"), lx, ly, lth, yaw, r.get("gz"), cur["wheel_scale"], cur["gyro_scale"],
                        cur["gyro_drift_dps"], ys, seg, anchor])
    if not out:
        return None, {}, words
    out.sort(key=lambda r: (r[0], r[1]))
    counts = {}
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(ODO_COLS)
        for r in out:
            w.writerow(["%.4f" % r[0], iso(r[0]), r[1], r[2], r[3], "%.2f" % r[4], "%.2f" % r[5], "%.2f" % r[6],
                        "%.2f" % r[7]] + list(r[8:15]) + ["%g" % r[15], "%g" % r[16], "%g" % r[17], "%g" % r[18],
                                                         r[19], r[20]])
            counts[r[1]] = counts.get(r[1], 0) + 1
    return out_csv, counts, words


def export_all(mission, say=print):
    res = {"version": VERSION}
    p, c = camera_track(mission)
    res["camera_track"] = {"file": p, "rows": c}
    say("camera track: %s (%s)" % (p or "none: no camera this run", ", ".join("%s %d" % kv for kv in sorted(c.items()))))
    p2, c2, w2 = odometry_raw(mission)
    res["odometry_raw"] = {"file": p2, "rows": c2, "anchors": w2}
    say("wheel+gyro odometry, no camera corrections: %s (%s)" % (p2 or "none: no RAW rows",
                                                                ", ".join("%s %d" % kv for kv in sorted(c2.items()))))
    p3, n3 = stops(mission)
    res["stops"] = {"file": p3, "rows": n3}
    if p3:
        say("stops at points (arrival, dwell start / end, camera position during the dwell): %s (%d)" % (p3, n3))
    return res


STOP_COLS = ["robot", "n", "point", "arrive_unix", "arrive_iso", "dwell_start_unix", "dwell_end_unix", "dwell_end_iso",
             "dwell_s", "cam_x_cm", "cam_y_cm", "plus_minus_cm", "n_fixes", "source", "own_x_cm", "own_y_cm",
             "robot_wifi", "tag_id"]   # (11-09b, Jay-Sun: the run's slot name, e.g. Alpha, is not the robot: its WiFi name too)


def robot_ids(mission):
    """(11-09b) {run name (slot or alias): (WiFi name, roof tag id)} from the wiggle binding (bind/binding.json)."""
    out = {}
    for p in (os.path.join(mission, "bind", "binding.json"), os.path.join(mission, "binding.json")):
        try:
            with open(p) as f:
                rec = json.load(f)
        except (OSError, ValueError):
            continue
        for r in rec.get("robots") or []:
            w, t = r.get("wifi_name") or r.get("name_before") or "", r.get("tag_id")
            for k in (r.get("slot"), r.get("name_before")):
                if k:
                    out[str(k)] = (w, t)
        for a, sl in (rec.get("aliases") or {}).items():
            if str(sl) in out:
                out.setdefault(str(a), out[str(sl)])
        break
    return out


def stops(mission):
    """(e11, Jay-Sun 9 Nov: "a stop has to be clearly findable in the logs") run/stops.csv: one row per stop at a point,
    in the order driven: arrival (= the dwell's start: the wait step starts the moment the drive to the point ends),
    dwell start and end on the Pi's clock (Unix seconds, NTP), and the camera's position during the dwell (the median
    of the camera fixes in it, the same numbers as points.csv). From the event log's point_confidence records."""
    run = os.path.join(os.path.abspath(mission), "run")
    src = os.path.join(run, "coordinator.jsonl")
    if not os.path.exists(src):
        return None, 0
    rows, k = [], {}
    ids = robot_ids(os.path.abspath(mission))
    with open(src, errors="replace") as f:
        for ln in f:
            if '"point_confidence"' not in ln:
                continue
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            t1 = e.get("t_abs")
            if t1 is None:
                continue
            t0 = float(t1) - float(e.get("dwell_s") or 0.0)
            r = str(e.get("robot") or "")
            k[r] = k.get(r, 0) + 1
            iso = lambda t: dt.datetime.fromtimestamp(t).astimezone().isoformat(timespec="milliseconds")  # noqa: E731
            rows.append({"robot": r, "n": k[r], "point": str(e.get("point") or "").replace("point ", ""),
                         "arrive_unix": "%.3f" % t0, "arrive_iso": iso(t0), "dwell_start_unix": "%.3f" % t0,
                         "dwell_end_unix": "%.3f" % float(t1), "dwell_end_iso": iso(float(t1)),
                         "dwell_s": e.get("dwell_s"), "cam_x_cm": e.get("x"), "cam_y_cm": e.get("y"),
                         "plus_minus_cm": e.get("plus_minus_cm"), "n_fixes": e.get("n_fixes"),
                         "source": e.get("source"), "own_x_cm": e.get("own_x"), "own_y_cm": e.get("own_y"),
                         "robot_wifi": (ids.get(r) or ("", ""))[0], "tag_id": (ids.get(r) or ("", ""))[1]})
    if not rows:
        return None, 0
    out = os.path.join(run, "stops.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=STOP_COLS)
        w.writeheader()
        w.writerows(rows)
    return out, len(rows)


# ---------------------------------------------------------------- the raw-files bundle for Jay-Sun
BUNDLE_FILES = [
    ("run/camera_track.csv", "every camera fix of each robot, whole run (this file's README)"),
    ("run/odometry_raw.csv", "wheel+gyro dead reckoning, never camera-corrected"),
    ("run/raw.csv", "the robots' RAW rows (rawlog.h columns; 50 a second)"),
    ("run/follow.csv", "the robots' own fused pose (wheels + gyro + camera fixes), FOLLOW records"),
    ("run/step.csv", "each step's queued / started / done with the robot's pose"),
    ("run/points.csv", "each production point: position, +- (95 %), source"),
    ("run/stops.csv", "each stop at a point, in order: arrival, dwell start / end (Unix s, the Pi's NTP clock), camera "
                      "position during the dwell (e11)"),
    ("visit_order.json", "Visit points in an order: the order asked, the points flagged unreadable, the order driven "
                         "(e11)"),
    ("run/clock_sync.csv", "robot clock (ms) against the Pi clock"),
    ("run/param.csv", "the robots' settings (wheel_scale, gyro_scale, ...)"),
    ("run/runinfo.csv", "firmware version, yaw_sign"),
    ("run/coordinator.jsonl", "the event log (every step, re-plan, keep-going event)"),
    ("run/experiment.json", "experiment id, orders, swap, seed, pinned speeds, sync wiggle times"),
    ("run/run_plan_summary.json", "run_plan's summary"),
    ("run/plan_given.json", "the plan as given"),
    ("run/messages.csv", "the robots' messages"),
    ("bridge/tag_poses.csv", "every camera read (per camera, before fusing)"),
    ("bridge/fixes.csv", "the camera fixes sent to the robots (at most 10 a second each)"),
    ("bridge/track.csv", "the camera bridge's track"),
    ("mission_summary.json", "the run's summary (experiment, sync wiggle, points)"),
    ("program.json", "the production program (orders)"),
    ("mission.json", "the mission (map with points, homes, keep-outs)"),
    ("sync_marks.jsonl", "start / end marks and Mission Control's marks (Pi clock)"),
    ("pose_timeline.csv", "camera and own poses on one timeline"),
    ("tags.json", "roof tag ids"),
    ("bind/binding.json", "which robot (WiFi name, e.g. AL-1DC0) was which run name (e.g. Alpha) and roof tag (11-09b)"),
    ("frames_where.txt", "where this run's camera pictures are (the USB stick when it is plugged in) (11-09b)"),
]
NEVER = ("rfid", "jay", ".zip")      # nothing from Jay-Sun's RFID capture or the tools/rfid outputs goes in


README = """Run %(session)s: raw files from the robots and cameras (Nick's MIRLT fleet), made %(made)s by run_exports.py %(ver)s.
Nothing from the RFID reader is in here. All times: Unix seconds on the Pi's clock (t), with the same moment as text (t_iso).
Units: cm and degrees. Frame: (0, 0) = corner 1 (tag 20, the A3/TV corner); x along the 156 in edge to the door end;
y toward the whiteboard; heading degrees counter-clockwise from +x. The middle line is x = 198.1 cm (78 in).

camera_track.csv: every moment a camera saw a robot (whole run). x_cm, y_cm, heading_deg: the roof tag's centre (= the
  robot's turning point) fused over the cameras that saw it within 0.07 s; sigma_cm / sigma_deg: the tag read's own 1-sigma
  (it does NOT include the table frame's error; the +- at each point, in points.csv, does); pm95_cm = 1.96 x sigma_cm;
  cams: which camera(s); n_reads; cams_apart_cm: how far the two cameras' reads were apart; source.
odometry_raw.csv: wheel + gyro dead reckoning per robot, NEVER corrected by a camera: anchored once at the robot's first
  own pose (its home) and then integrated from the RAW rows as the firmware does (distance from the wheels x wheel_scale,
  heading from the gyro yaw x yaw_sign x gyro_scale - drift). x_odo_cm, y_odo_cm, heading_odo_deg, dist_cm; the raw
  values used (wl_deg, wr_deg wheel angles; lib_x/y/th the motor board's own odometry; yaw_deg, gz gyro) and the
  corrections in force; segment / anchor (a new segment only if the robot restarted).
raw.csv: rawlog.h columns (v_cms is mm/s; rpm/v/w read ~1.08x high; see the repo's tools/raw_units.py).
follow.csv: the robot's own fused pose (x_cm, y_cm, th_deg) WITH camera fixes: compare with odometry_raw.csv.
experiment.json / mission_summary.json: experiment id, orders, halves swapped, seed, pinned speeds, sync wiggle start/end
  per robot (Pi clock; each robot: quarter turn, 1 s stop, quarter turn back, 1 s stop, at its home).
Sync wiggle times: %(wiggle)s
"""


def bundle(mission, out_zip=None):
    """The raw-files zip for Jay-Sun in the mission folder (never committed: logs/ is git-ignored). Returns
    (path, [names in it])."""
    mission = os.path.abspath(mission)
    sess = os.path.basename(mission.rstrip("/"))
    run = os.path.join(mission, "run")
    if not os.path.exists(os.path.join(run, "camera_track.csv")) and not os.path.exists(
            os.path.join(run, "odometry_raw.csv")):
        export_all(mission, say=lambda s: None)
    out_zip = out_zip or os.path.join(mission, "jay-raw-%s.zip" % sess)
    wig = "none logged"
    try:
        with open(os.path.join(mission, "mission_summary.json")) as f:
            sw = json.load(f).get("sync_wiggle") or {}
        if sw:
            wig = "; ".join("%s %.3f -> %.3f" % (n, v["start"], v["end"]) for n, v in sorted(sw.items()))
    except (OSError, ValueError):
        pass
    names = []
    with zipfile.ZipFile(out_zip, "w", zipfile.ZIP_DEFLATED) as z:
        for rel, _what in BUNDLE_FILES:
            p = os.path.join(mission, rel)
            if not os.path.exists(p) or any(k in rel.lower() for k in NEVER):
                continue
            z.write(p, os.path.join(sess, rel))
            names.append(rel)
        # the site file and the typed points (the frame and the stickers)
        try:
            with open(os.path.join(mission, "mission_cmd.json")) as f:
                site = (json.load(f).get("args") or {}).get("site") or "school-conference"
        except (OSError, ValueError):
            site = "school-conference"
        for rel in ("sites/%s.json" % site, "sites/%s-points.csv" % site):
            p = os.path.join(HERE, rel)
            if os.path.exists(p):
                z.write(p, os.path.join(sess, "site", os.path.basename(rel)))
                names.append("site/" + os.path.basename(rel))
        lst = "\n".join("  %s: %s" % (r, w) for r, w in BUNDLE_FILES if r in names)
        z.writestr(os.path.join(sess, "README.txt"), README % {
            "session": sess, "made": dt.datetime.now().astimezone().isoformat(timespec="seconds"), "ver": VERSION,
            "wiggle": wig} + "\nFiles:\n" + lst + "\n  site/: the site file and the typed sticker points\n")
        names.append("README.txt")
    return out_zip, names


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission", help="the mission folder (logs/...-mission or Mission Control's runs/<...>), or latest "
                                    "(the school site's newest production run)")
    ap.add_argument("--bundle", action="store_true", help="also the raw-files zip for Jay-Sun (in the mission folder)")
    a = ap.parse_args(argv)
    if a.mission == "latest":
        # the newest production run of the school site's Mission Control (as tools/rfid/school_rfid.py's latest)
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "rfid"))
        import school_rfid as SR
        a.mission = SR.latest_run()
        print("the newest run: %s" % a.mission)
    m = os.path.abspath(a.mission)
    if os.path.basename(m) == "run":
        m = os.path.dirname(m)
    if not os.path.isdir(os.path.join(m, "run")):
        raise SystemExit("%s has no run/ folder: give the mission's folder (Review shows it)" % m)
    export_all(m)
    if a.bundle:
        p, names = bundle(m)
        print("bundle for Jay-Sun: %s (%d files, %.1f MB)" % (p, len(names), os.path.getsize(p) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
