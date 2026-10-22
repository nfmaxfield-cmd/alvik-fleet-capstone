"""Studio glue for outside data (outside.py, robotlogs.py) and home positions (homes.py).

The studio keeps, per recording, in labels/<recording>/:
    outside.json          where the outside data is (folder or file) and which plug-in reads it
    outside_drafts.json   the guesses made from it last time ("Line up again" makes them anew), compact:
                          {"frames": {frame: [[name, cls, x, y, heading or null, heading_known 0/1, score,
                          sd_px], ...]}, "names": {obj: name}, "summary": [...], "made": time, ...}

Nothing here knows about Alviks: the plug-in reads the log; the Alvik dock is one preset in homes.py.

    floor_camera(proj, rec)              -> (camera model, path) or (None, None): the floor calibration
    NO_FLOOR                             what to say when there is none
    plugin_list()                        [(id, title, description)]
    read_settings / save_settings / clear_settings
    build(proj, rec, session, say)       read, line up and make the guesses (writes outside_drafts.json)
    load_drafts(proj, rid)               -> the Session's `outside` argument, or None
    split_voter(frames, robot_px)        the tracker's split hook from the drafts
    home_refiner(homes, camera_key)      the Session's `refine` argument: home visits as free confirmations
"""
import json
import math
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))

SETTINGS_FILE = "outside.json"
DRAFTS_FILE = "outside_drafts.json"
BEFORE = ".before"           # outside_drafts.json as it was before the last Line up (swap_drafts puts it back)
HOME_SCORE = 0.97            # a guess from a home visit: the place and facing are known exactly
NO_FLOOR = ("This needs the floor calibration of camera %s (which pixel is which spot on the floor), and there is "
            "none yet. Make one with More, then Floor calibration: click the 4 corners of something rectangular "
            "of known size lying on the floor (a sheet of paper, a board) and type its width and length. It takes "
            "a minute. Then come back here. Nothing was changed. (If your positions are already in picture pixels, "
            "no calibration is needed: tick \"My positions are in picture pixels\" instead.)")
UNITS = ("cm", "px")


def _write(path, data):
    import project as projmod
    projmod.write_json(path, data)


def _read(path, default=None):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


# =============================================================================== the floor calibration
def floor_paths(proj, rec):
    """Where a recording's floor calibration may be, in order: the project's camera file (cameras/<camera
    name or default>.json), then a camera.json or floor_calibration.json in the recording folder."""
    key = proj.camera_key(rec)
    out = [proj.camera_model_path(key)]
    if rec.get("camera"):
        out.append(proj.camera_model_path(rec["camera"]))
    out += [os.path.join(rec["path"], "camera.json"), os.path.join(rec["path"], "floor_calibration.json")]
    seen, keep = set(), []
    for p in out:
        if p and p not in seen:
            seen.add(p)
            keep.append(p)
    return keep


class PixelMap:
    """"Floor" = the picture itself, for positions given in picture pixels (x right, y down from the top-left
    corner; no floor calibration needed). Heights are ignored; one pixel counts as one unit."""

    def __init__(self, image_size=None):
        self.image_size = tuple(image_size) if image_size else None

    def project(self, xyz):
        import numpy as np
        p = np.asarray(xyz, float)
        if p.ndim == 1:
            return float(p[0]), float(p[1])
        p = p.reshape(-1, p.shape[-1])
        return p[:, 0].copy(), p[:, 1].copy()

    def to_floor(self, u, v, z=0.0):
        import numpy as np
        if np.ndim(u) == 0:
            return float(u), float(v)
        return np.asarray(u, float).copy(), np.asarray(v, float).copy()

    def in_image(self, u, v, margin=0.0):
        import numpy as np
        u, v = np.asarray(u, float), np.asarray(v, float)
        ok = np.isfinite(u) & np.isfinite(v)
        if self.image_size:
            W, H = self.image_size
            ok &= (u >= -0.5 + margin) & (u <= W - 0.5 - margin) & (v >= -0.5 + margin) & (v <= H - 0.5 - margin)
        return ok

    def px_per_cm(self, x, y, z=0.0):
        return 1.0


def image_size_of(rec, session=None):
    s = getattr(session, "rec", None)
    if s is not None and getattr(s, "width", None):
        return (int(s.width), int(s.height))
    m = _read(os.path.join(rec["path"], "meta.json"), {}) or {}
    if m.get("width") and m.get("height"):
        return (int(m["width"]), int(m["height"]))
    return None


def floor_camera(proj, rec, settings=None, session=None):
    """The recording's floor map: a PixelMap when its outside data says its positions are in picture
    pixels (settings units "px"), else the floor calibration (camera model, path) or (None, None)."""
    import outside
    if settings and settings.get("units") == "px":
        return PixelMap(image_size_of(rec, session)), "picture pixels (no floor calibration needed)"
    for p in floor_paths(proj, rec):
        if os.path.exists(p):
            try:
                return outside.load_floor_map(p), p
            except Exception:
                continue
    return None, None


def no_floor_text(proj, rec):
    return NO_FLOOR % proj.camera_key(rec)


def clock_info(rec):
    """The recording's own clock, for the page: first and last t_wall (seconds), whether it was estimated
    (a video file: from the file's time), and plain words saying which clock a positions file uses."""
    times = _frame_times(rec)
    meta = _read(os.path.join(rec["path"], "meta.json"), {}) or {}
    estimated = meta.get("device") == "video file" or "video file" in str(meta.get("made_by") or "")
    if not times:
        return dict(ok=False, estimated=estimated, text=(
            "This recording has no frames.csv, so it has no clock to line outside data up with."))
    fs = sorted(times)
    t0, t1 = times[fs[0]], times[fs[-1]]
    import datetime as dtm
    when = dtm.datetime.fromtimestamp(t0).strftime("%Y-%m-%d %H:%M:%S")
    text = ("Times are on this recording's clock: t_wall in seconds (seconds since 1 January 1970, as the t_wall column "
            "of the recording's frames.csv). This recording runs from %.2f to %.2f (%s, %.0f s long). A positions "
            "file's t column must be on the same clock." % (t0, t1, when, t1 - t0))
    if estimated:
        text += (" This recording was made from a video file, so its clock was estimated from the file's time (the "
                 "moment the file was saved, taken as the end of the recording): it can be off by seconds. If the "
                 "guesses run ahead of or behind the objects, use Line up by a moment below.")
    return dict(ok=True, t_first=round(t0, 3), t_last=round(t1, 3), start=when, seconds=round(t1 - t0, 2),
                estimated=estimated, text=text, n_frames=len(fs))


def offset_from_moment(proj, rec, settings, frame, log_time, kind="file"):
    """The time offset that makes the data's moment log_time line up with frame: the person picked a frame
    on video where they know the data's time. kind "file": log_time as written in the data (its own time
    column); "from_start": seconds after the data's first sample. Returns (offset_s, plain words)."""
    times = _frame_times(rec)
    if int(frame) not in times:
        raise ValueError("Frame %s has no time in this recording's frames.csv. Nothing was changed." % frame)
    t_frame = times[int(frame)]
    try:
        lt = float(log_time)
    except (TypeError, ValueError):
        raise ValueError("Type the data's time at this frame as a number of seconds (%r is not). Nothing was "
                         "changed." % (log_time,))
    data0 = read_data(proj, rec, settings["path"], settings.get("plugin"), 0.0)
    if not data0.streams:
        raise ValueError("The outside data holds no positions. Nothing was changed.")
    starts = [s.t[0] for s in data0.streams.values() if len(s)]
    if kind == "from_start":
        t_data = min(starts) + lt
        how = "%.2f s after the data's first sample" % lt
    else:
        conv = getattr(data0, "to_wall", None)
        t_data = float(conv(lt)) if callable(conv) else lt
        how = "the data's time %s" % ("%.3f" % lt).rstrip("0").rstrip(".")
    off = t_frame - t_data
    words = ("Frame %d (t_wall %.2f) is %s, so the data's times are moved by %+.2f s." % (int(frame), t_frame, how, off))
    return round(off, 3), words


# =============================================================================== settings per recording
def plugin_list():
    import outside
    return [dict(id=i, title=t, description=d) for i, t, d in outside.plugins()]


TIME_WORDS = {"csv": "the t column of the positions file (seconds)",
              "alvik": "the laptop's time in raw.csv (the t_recv column, seconds)"}


def time_words(plugin_id):
    """Which of the data's own times a person types for Line up by a moment."""
    return TIME_WORDS.get(plugin_id, "the time as written in the data (seconds)")


def plugin_words(plugin_id):
    """What the guesses are called on the page ("Guesses from motion and robot logs")."""
    return {"alvik": "robot logs", "csv": "the positions file"}.get(plugin_id, "the positions log")


def settings_path(proj, rid):
    return os.path.join(proj.labels_dir(rid), SETTINGS_FILE)


def read_settings(proj, rid):
    return _read(settings_path(proj, rid))


def save_settings(proj, rid, path, plugin=None, offset_s=0.0, units="cm"):
    """units: "cm" (floor positions, through the floor calibration) or "px" (positions already in picture
    pixels). Older files have no units: they mean cm."""
    os.makedirs(proj.labels_dir(rid), exist_ok=True)
    if units not in UNITS:
        raise ValueError("units must be cm or px")
    s = dict(path=os.path.abspath(os.path.expanduser(path)), plugin=plugin or None, offset_s=float(offset_s or 0.0),
             added=time.strftime("%Y-%m-%dT%H:%M:%S"))
    if units == "px":
        s["units"] = "px"
    _write(settings_path(proj, rid), s)
    return s


def clear_settings(proj, rid):
    """Stop using outside data for this recording (both files are kept as .off, so it can be put back)."""
    moved = []
    for name in (SETTINGS_FILE, DRAFTS_FILE):
        p = os.path.join(proj.labels_dir(rid), name)
        if os.path.exists(p):
            os.replace(p, p + ".off")
            moved.append(name)
    return moved


def restore_settings(proj, rid):
    """Undo of clear_settings."""
    back = []
    for name in (SETTINGS_FILE, DRAFTS_FILE):
        p = os.path.join(proj.labels_dir(rid), name)
        if os.path.exists(p + ".off"):
            os.replace(p + ".off", p)
            back.append(name)
    return back


def read_data(proj, rec, path, plugin=None, offset_s=0.0, units="cm"):
    """Read the outside data with its plug-in (plain ValueError when it can't). units "px": the positions
    are picture pixels (x right, y down; headings as on the labelling page: 0 = right, counter-clockwise on
    screen); only positions on the floor itself (not a robot's own path) can be given that way."""
    import outside
    path = os.path.abspath(os.path.expanduser((path or "").strip().strip('"')))
    if not os.path.exists(path):
        raise ValueError("There is nothing at %s. Nothing was changed." % path)
    opts = dict(recording=rec["path"])
    if offset_s:
        opts["offset_s"] = float(offset_s)
    try:
        data = outside.read_outside(path, plugin or None, **opts)
        if units == "px":
            own = [n for n, st in data.streams.items() if st.frame != "floor"]
            if own:
                raise ValueError("Positions in picture pixels must be where each object is in the picture; %s %s its "
                                 "own path from where it started (like a robot's wheels), which needs the floor "
                                 "calibration. Untick \"My positions are in picture pixels\"" % (
                                     ", ".join(own[:3]), "is" if len(own) == 1 else "are"))
            for st in data.streams.values():
                st.heading = -st.heading            # picture convention (counter-clockwise on screen, y down)
        return data
    except KeyError as e:
        raise ValueError("No such kind of outside data: %s. Nothing was changed." % e)
    except TypeError as e:          # a plug-in that does not take one of the settings (e.g. the offset)
        raise ValueError("This kind of outside data can't use one of the settings given (%s). Clear the time "
                         "offset box, or pick the kind of data by hand. Nothing was changed." % e)
    except ValueError as e:
        raise ValueError("%s. Pick the kind of data by hand, or check the folder. Nothing was changed." % e)


# =============================================================================== building the guesses
def _frame_times(rec):
    import crosscam
    p = os.path.join(rec["path"], "frames.csv")
    if not os.path.exists(p):
        return {}
    return crosscam.load_frame_times(p)


def _cls_for(proj, stream):
    names = proj.class_names()
    if stream.cls in names:
        return stream.cls
    t = proj.motion_object_type()
    return t if t in names else ("unknown" if "unknown" in names else names[0])


def build(proj, rec, session, say=print):
    """Read the recording's outside data, match each stream to a followed object (roll call, by speed
    over time), line it up on the floor (home visits, the log's own "at home" events, the person's
    labels, the first forward stretch seen on video), and make a guess for every frame. Writes
    outside_drafts.json and returns its summary. Raises ValueError with plain words."""
    import numpy as np
    import outside
    import homes as homesmod
    st = read_settings(proj, rec["id"])
    if not st:
        raise ValueError("No outside data is set for this recording.")
    units = st.get("units") or "cm"
    if units == "px":
        cam, cam_path = PixelMap(image_size_of(rec, session)), "picture pixels (no floor calibration needed)"
    else:
        cam, cam_path = floor_camera(proj, rec)
    if cam is None:
        raise ValueError(no_floor_text(proj, rec))
    times = _frame_times(rec)
    if not times:
        raise ValueError("This recording has no shared clock (frames.csv with t_wall), so outside data can't be lined "
                         "up with its frames. Recordings from the Pi have it; a video file made into a recording does "
                         "not. Nothing was changed.")
    say("Reading the outside data ...")
    data = read_data(proj, rec, st["path"], st.get("plugin"), st.get("offset_s"), units)
    if not data.streams:
        raise ValueError("The outside data holds no positions (%s). Nothing was changed." % data.summary())
    heights = proj.class_heights()
    cls_of = {n: _cls_for(proj, s) for n, s in data.streams.items()}
    key = proj.camera_key(rec)

    # the followed objects on the floor (the person's labels win over the guesses on their frames)
    say("Matching the outside data to the objects on video (roll call) ...")
    # a copy under the session's lock: the person may be labelling while this runs (after any re-track
    # still running in the background has finished, so the guesses are whole)
    if hasattr(session, "wait_settled"):
        session.wait_settled(120)
    lock = getattr(session, "lock", None)
    if lock is not None:
        lock.acquire()
    try:
        guesses = {f: [dict(r) for r in rs] for f, rs in session.guesses.items()}
        frames = {f: dict(stt, objects=[dict(o) for o in stt["objects"]]) for f, stt in session.frames.items()}
    finally:
        if lock is not None:
            lock.release()
    rows = {}
    for f, rs in guesses.items():
        rows[f] = rs
    for f, stt in frames.items():
        if stt["state"] in ("edited", "reviewed"):
            rows[f] = [o for o in stt["objects"] if o["status"] != "rejected"]
    tracks_px = {}
    for f in sorted(rows)[::2]:
        t = times.get(f)
        if t is None:
            continue
        for o in rows[f]:
            tracks_px.setdefault(o["obj"], []).append((t, o["x"], o["y"], o["cls"]))
    tracks_cm = {}
    for obj, pts in tracks_px.items():
        if len(pts) < 8:
            continue
        z = heights.get(pts[0][3], 0.0)
        tracks_cm[obj] = outside.track_floor([(p[0], p[1], p[2]) for p in pts], cam, z)
    names, _detail = outside.match_names(data.streams, tracks_cm) if tracks_cm else ({}, {})
    clock_found = None
    if not getattr(data, "clock_known", True) and names:
        # nothing ties the data's clock to the video's: work the offset out from how fast each matched
        # object moved on video and in the data (it does not need the floor alignment)
        say("Working out the outside data's clock from how fast each object moved ...")
        clock_found = find_offset(data, tracks_cm, names)
        if clock_found and clock_found["offset_s"] is not None:
            data = read_data(proj, rec, st["path"], st.get("plugin"),
                             float(st.get("offset_s") or 0.0) + clock_found["offset_s"], units)
            names, _detail = outside.match_names(data.streams, tracks_cm)
    # every followed object that is clearly one of the streams (a person's labels give the same object new
    # numbers over a long recording, and each of those labels should line the data up); each name's main
    # object is the one seen matching longest, so a short stretch can't take the name at the next Line up
    names, all_names = name_objects(data.streams, tracks_cm, names)
    obj_of = {n: o for o, n in names.items()}

    # anchors: home visits and "at home" events, the person's labels, a forward stretch seen on video
    hl = homesmod.homes_of(proj.data, key)
    anchors = {}

    def add(d):
        for n, lst in d.items():
            anchors.setdefault(n, []).extend(lst)
    if hl:
        tr_home = homesmod.tracks_from_labels(rows, times)
        visits = homesmod.find_visits(hl, tr_home, camera=key, obj_names=names)
        z = heights.get(next(iter(cls_of.values())), 0.0)      # the streams' type (one height for the homes)
        add(homesmod.anchors_from_visits(visits, hl, cam, obj_names=names, height_cm=z, outside=data))
        add(homesmod.events_to_anchors(data.events, hl, cam, height_cm=z))
    labelled = {f: v for f, v in frames.items() if v["state"] in ("edited", "reviewed")}
    add(outside.anchors_from_labels(labelled, times, cam, all_names, heights))
    for n, s in data.streams.items():
        if s.frame == "own" and n in obj_of and obj_of[n] in tracks_cm:
            add({n: outside.heading_anchor_from_motion(s, tracks_cm[obj_of[n]])})
    say("Lining the outside data up with the floor ...")
    als = outside.align(data, anchors)
    say("Making a guess for every frame ...")
    drafts = outside.draft_labels(als, times, cam, obj_names=obj_of, cls_of=cls_of, class_heights=heights,
                                  plugin=data.plugin)
    compact, n = {}, 0
    for f, ds in drafts.items():
        lst = []
        for d in ds:
            z = heights.get(d["cls"], 0.0)
            try:
                ppc = cam.px_per_cm(d["x_cm"], d["y_cm"], z)
            except Exception:
                ppc = 1.0
            hd = d["heading_deg"] if d["heading_known"] else d.get("heading_guess")
            lst.append([d["name"], d["cls"], d["x"], d["y"], hd, 1 if d["heading_known"] else 0, d["score"],
                        round(max(1.0, d["sd_cm"] * ppc), 2)])
            n += 1
        compact[str(f)] = lst
    placed = {}
    for ds in drafts.values():
        for d in ds:
            placed[d["name"]] = placed.get(d["name"], 0) + 1
    summary = []
    for nme, al in sorted(als.items()):
        kinds = {}
        for a in al.anchors:
            kinds[a.kind] = kinds.get(a.kind, 0) + 1
        summary.append(dict(name=nme, cls=cls_of[nme], obj=obj_of.get(nme), anchors=kinds, frames=placed.get(nme, 0),
                            text=al.describe()))
    out = dict(made=time.strftime("%Y-%m-%dT%H:%M:%S"), plugin=data.plugin, path=st["path"], camera_file=cam_path,
               names={str(o): nm for o, nm in all_names.items()}, summary=summary,
               notes=list(data.notes) + ([clock_found["text"]] if clock_found else []) +
               [al.calib_note for _, al in sorted(als.items()) if getattr(al, "calib_note", None)],
               clock=data.clock, clock_found=clock_found, n=n, labelled_frames=len(labelled), frames=compact,
               units=units, offset_s=float(st.get("offset_s") or 0.0))
    dp = os.path.join(proj.labels_dir(rec["id"]), DRAFTS_FILE)
    if os.path.exists(dp):              # the guesses made last time: "Line up again" can be taken back (Z)
        import shutil
        shutil.copyfile(dp, dp + BEFORE)
    _write(dp, out)
    return {k: v for k, v in out.items() if k != "frames"}


def name_objects(streams, tracks_cm, primary, clear=0.6, max_cost=3.0, min_travel_cm=10.0, max_overlap=0.2):
    """Names for the followed objects. Returns (main, every): every = {obj: name} for each object whose speed
    over time clearly matches one stream (mean speed difference under max_cost cm/s and under `clear` x the
    next stream's; an object that barely moved looks like every stream at rest and is left out), never two
    objects with one name on video at the same time (the one seen matching longer wins); main = {obj: name}
    with one object per name, the one seen matching longest. `primary` (outside.match_names: one object per
    name, by lowest cost alone, so a short stretch can win) is used for a name nothing clearly matched."""
    import numpy as np
    import outside
    span = {o: (float(tr[0, 0]), float(tr[-1, 0])) for o, tr in tracks_cm.items() if len(tr)}
    cand = []
    for obj, tr in tracks_cm.items():
        if len(tr) < 2 or float(np.sum(np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])))) < min_travel_cm:
            continue
        _, det = outside.match_names(streams, {obj: tr})
        costs = sorted((c, n) for (_, n), c in det.items())
        if not costs or costs[0][0] >= max_cost:
            continue
        if len(costs) > 1 and costs[0][0] >= clear * costs[1][0]:
            continue
        name = costs[0][1]
        st = streams[name]
        cover = max(0.0, min(span[obj][1], float(st.t[-1])) - max(span[obj][0], float(st.t[0])))
        cand.append((-cover, costs[0][0], obj, name))

    def overlap(a, b):
        (a0, a1), (b0, b1) = span[a], span[b]
        return min(a1, b1) - max(a0, b0) > max_overlap * max(1e-6, min(a1 - a0, b1 - b0))
    every, main = {}, {}
    for _, _, obj, name in sorted(cand):
        if not any(nm == name and overlap(obj, o) for o, nm in every.items()):
            every[obj] = name
            if name not in main.values():
                main[obj] = name
    for obj, name in primary.items():
        if name not in main.values() and obj not in every and obj in span and \
                not any(nm == name and overlap(obj, o) for o, nm in every.items()):
            main[obj] = name
            every[obj] = name
    return main, every


def find_offset(data, tracks_cm, names, min_corr=0.5):
    """The outside data's clock offset from speed over time (outside.estimate_time_offset) for every
    stream matched to a followed object; the median of the clear ones. Returns dict(offset_s, used,
    tried, corr, text) with plain words; offset_s None when no stream gave a clear answer."""
    import numpy as np
    import outside
    got = []
    for obj, name in names.items():
        stv = data.streams.get(name)
        tr = tracks_cm.get(obj)
        if stv is None or tr is None:
            continue
        off, c = outside.estimate_time_offset(stv, tr)
        if off is not None and c >= min_corr:
            got.append((off, c, name))
    if not got:
        return dict(offset_s=None, used=0, tried=len(names), corr=None,
                    text=("The outside data's clock is not tied to the video's (no clock file came with it), and how "
                          "the objects moved did not show a clear time difference either, so the times were used as "
                          "they are. If the guesses run ahead of or behind the objects, type the difference in the "
                          "time offset box and press Line up again."))
    offs = sorted(g[0] for g in got)
    off = float(np.median(offs))
    spread = (offs[-1] - offs[0]) if len(offs) > 1 else 0.0
    way = "behind" if off > 0 else "ahead of"
    text = ("The outside data's clock is not tied to the video's (no clock file came with it), so it was worked "
            "out from how fast each object moved on video and in the data: the data runs %.2f s %s the video "
            "(from %d of %d object%s%s). Its times were moved by %+.2f s. To set it yourself instead, type the "
            "difference in the time offset box and press Line up again." % (
                abs(off), way, len(got), len(names), "" if len(names) == 1 else "s",
                ", agreeing within %.2f s" % spread if len(got) > 1 else "", off))
    return dict(offset_s=round(off, 3), used=len(got), tried=len(names), corr=round(float(np.mean([g[1] for g in got])), 3),
                spread_s=round(spread, 3), text=text)


def swap_drafts(proj, rid):
    """Undo / redo of "Line up again": swap outside_drafts.json with the guesses made the time before.
    Returns the summary now in use, or raises ValueError when there is nothing to swap with."""
    dp = os.path.join(proj.labels_dir(rid), DRAFTS_FILE)
    bp = dp + BEFORE
    if not (os.path.exists(dp) and os.path.exists(bp)):
        raise ValueError("The guesses from before the last Line up are not kept any more (the outside data was "
                         "stopped or used anew since). Nothing was changed.")
    tmp = dp + ".swap"
    os.replace(dp, tmp)
    os.replace(bp, dp)
    os.replace(tmp, bp)
    return drafts_info(proj, rid)


def drafts_info(proj, rid):
    d = _read(os.path.join(proj.labels_dir(rid), DRAFTS_FILE))
    if not d:
        return None
    return {k: v for k, v in d.items() if k != "frames"}


def load_drafts(proj, rid):
    """The Session's `outside` argument from outside_drafts.json (None when there is none)."""
    d = _read(os.path.join(proj.labels_dir(rid), DRAFTS_FILE))
    if not d or not d.get("frames"):
        return None
    frames = {}
    for f, lst in d["frames"].items():
        frames[int(f)] = [dict(name=e[0], cls=e[1], x=float(e[2]), y=float(e[3]), heading_deg=e[4],
                               heading_known=bool(e[5]), score=float(e[6]), sd_px=float(e[7]), source="log")
                          for e in lst]
    label = plugin_words(d.get("plugin"))
    return dict(drafts=frames, make_vote=lambda r: split_voter(frames, r), label=label,
                info={k: v for k, v in d.items() if k != "frames"})


def split_voter(frames, robot_px):
    """The tracker's split hook: when lookalikes came together and parted, which reading the outside data
    supports. Each identity is named by the draft nearest its place just before they came together; each
    path is compared with that name's drafts after. Cost 0.5 * mean((error / sd)^2), at most 5."""
    lim = 0.75 * float(robot_px)

    def near(f, x, y):
        for g in (f, f - 1, f + 1, f - 2, f + 2):
            ds = frames.get(g)
            if not ds:
                continue
            best = min(ds, key=lambda d: math.hypot(d["x"] - x, d["y"] - y))
            if math.hypot(best["x"] - x, best["y"] - y) <= lim:
                return best["name"]
        return None

    def vote(f_before, idents, paths):
        names = {i: near(f_before, x, y) for i, (x, y) in idents.items()}
        named = {i: n for i, n in names.items() if n}
        if len(set(named.values())) < 2 or len(set(named.values())) != len(named):
            return None
        costs = {}
        for i, nm in named.items():
            for pid, pts in paths.items():
                errs = []
                for (f, x, y) in pts[::2]:
                    d = next((d for d in frames.get(f, ()) if d["name"] == nm), None)
                    if d is not None:
                        errs.append(((x - d["x"]) ** 2 + (y - d["y"]) ** 2) / (d.get("sd_px", 2.0) ** 2 + 4.0))
                if errs:
                    costs[(i, pid)] = min(5.0, 0.5 * sum(errs) / len(errs))
        return costs or None
    return vote


# =============================================================================== home positions
# Ready-made home positions (the page shows these words; homes.py does the geometry). Generic ones first; the
# Alvik dock is the Alvik plug-in's (shown when the project has an "alvik" type). clicks: what to click, in order
# (the page asks for each); needs_floor: needs the camera's floor calibration; facing: "ask" (a click for it),
# "none", or "option" (a tick box: the objects face along the first side).
HOME_PRESETS = [
    dict(id="spot", title="A fixed spot", name="Spot", kind="generic", needs_floor=False, facing="none",
         clicks=["Click the middle of the spot where the object sits when it is home."],
         added="one home position, no facing (the object can sit there any way round)",
         description="One click: a parking place, a mark on the floor, a fixture's seat."),
    dict(id="spot_facing", title="A spot with a facing", name="Home", kind="generic", needs_floor=False, facing="ask",
         clicks=["Click the middle of the spot where the object sits when it is home.",
                 "Now click where it faces when it is home."],
         added="one home position with its facing",
         description="Two clicks: where it sits, then the way it faces (only for object types with a front)."),
    dict(id="tray", title="A tray or fixture with slots", name="Tray", kind="generic", needs_floor=False,
         facing="option", clicks=["Click corner 1 of the tray (any corner).", "Click corner 2: the next corner along one side.",
                                  "Click corner 3: the corner opposite corner 1.", "Click corner 4: the last corner."],
         added="one home position per slot, kept together as one item",
         description="Four corners, then how many rows and columns of slots: every slot becomes a home position."),
    dict(id="alvik_dock", title="Alvik dock", name="Dock", kind="alvik", for_whom="Alvik robots", needs_type="alvik",
         needs_floor=True,
         facing="fixed",
         owner_label="Robot that docks here",
         owner_title="The robot's name in its log; blank: whichever robot sits there",
         clicks=["Click the middle of the back wall's top inner edge (where a docked robot's nose touches).",
                 "Now click the middle of the funnel's mouth (the open end)."],
         click1="Click the middle of the back wall's top inner edge (where a docked robot's nose touches).",
         click2="Now click the middle of the funnel's mouth (the open end).",
         added="the parked place and the three spots by its posts where a robot stops briefly",
         description="Two clicks on the Alvik robot's dock: its parked place and the stops by its posts."),
]
for _p in HOME_PRESETS:
    _p.setdefault("owner_label", "Belongs to")
    _p.setdefault("owner_title", "The object that lives here, by the name its positions log uses. Blank: whoever sits there.")
TRAY_MAX_SLOTS = 48


def presets_for(class_names):
    """The presets a project is offered: the generic ones, and a plug-in's own (needs_type) only when the project
    has that object type."""
    names = set(class_names or ())
    return [p for p in HOME_PRESETS if not p.get("needs_type") or p["needs_type"] in names]


def home_preset(preset_id):
    for p in HOME_PRESETS:
        if p["id"] == preset_id:
            return p
    raise ValueError("No home-position preset %r." % preset_id)


def tray_homes(name, camera_key, corners, rows, cols, robot_px=30.0, facing=False, owner=None, cls=None):
    """A tray or fixture with rows x cols slots from its 4 corners (picture px, in order around): one home
    per slot at the slot's middle (exact for a flat tray seen by any camera), kept together as one group.
    facing True: every slot's object faces along the first side (corner 1 to corner 2)."""
    import numpy as np
    import cv2
    import homes as homesmod
    rows, cols = int(rows), int(cols)
    if rows < 1 or cols < 1:
        raise ValueError("A tray needs at least 1 row and 1 column of slots.")
    if rows * cols > TRAY_MAX_SLOTS:
        raise ValueError("At most %d slots in one tray (%d x %d = %d): make two trays." % (TRAY_MAX_SLOTS, rows, cols,
                                                                                          rows * cols))
    q = np.asarray(corners, float).reshape(-1, 2)
    if len(q) != 4:
        raise ValueError("Click the tray's 4 corners first.")
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    Hm = cv2.getPerspectiveTransform(sq, q.astype(np.float32))

    def at(a, b):
        p = Hm @ np.array([a, b, 1.0])
        return p[:2] / p[2]
    out = []
    letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    for i in range(rows):
        for j in range(cols):
            a, b = (j + 0.5) / cols, (i + 0.5) / rows
            c = at(a, b)
            cell = min(np.linalg.norm(at(a + 0.5 / cols, b) - at(a - 0.5 / cols, b)),
                       np.linalg.norm(at(a, b + 0.5 / rows) - at(a, b - 0.5 / rows)))
            r = max(4.0, min(0.35 * cell, 0.6 * float(robot_px or 30.0)))
            f = None
            if facing:
                d = at(a + 0.1 / cols, b) - c
                f = math.degrees(math.atan2(-d[1], d[0])) % 360.0
            slot = ("%s%d" % (letters[i % 26], j + 1)) if rows > 1 else str(j + 1)
            out.append(homesmod.make_home("%s %s" % (name, slot), camera_key, c[0], c[1], f, r, kind="park",
                                          owner=owner, cls=cls, preset="tray", group=name))
    return out


def preset_homes(preset_id, name, camera_key, cam, click1, click2, owner=None, version=2, stations=True, **kw):
    """A preset's home positions from its clicks (homes.py does the geometry). Alvik dock: click1 = the back
    wall, click2 = the mouth; tray: kw corners, rows, cols, facing, robot_px."""
    import homes as homesmod
    if preset_id == "alvik_dock":
        out = homesmod.alvik_dock_homes(name, camera_key, cam, click1, click2, owner=owner, version=version,
                                        stations=stations)
        for h in out:
            h["group"] = name
        return out
    if preset_id == "tray":
        return tray_homes(name, camera_key, kw.get("corners"), kw.get("rows") or 1, kw.get("cols") or 1,
                          robot_px=kw.get("robot_px") or 30.0, facing=bool(kw.get("facing")), owner=owner,
                          cls=kw.get("cls"))
    raise ValueError("No home-position preset %r." % preset_id)


def home_refiner(home_list, camera_key):
    """The Session's `refine` argument: an object sitting still at a home position is a free confirmation
    there (exact place and facing). Returns None when there are no homes.

    A visit counts only when the visitor fits the home: of the home's object type (when the project has
    that type), and, for a home with an owner, not named otherwise by the roll call (the outside data's
    names). The guesses it makes replace only guesses of that type at the spot (never other objects),
    its facing holds only while the object sits there, and they are never accepted without a person
    looking (Session.auto_accept_frames leaves frames with a home guess alone)."""
    hl = [h for h in home_list or [] if (h.get("camera") or "default") == camera_key]
    if not hl:
        return None

    def refine(session):
        import collections
        import homes as homesmod
        times = {f: session.rec.times[f] for f in range(session.rec.n)}
        tracks = homesmod.tracks_from_labels(session.guesses, times)
        visits = homesmod.find_visits(hl, tracks, camera=camera_key)
        by = {h["name"]: h for h in hl}
        dets, out, skipped = {}, [], []
        for v in visits:
            h = by[v["home"]]
            span = range(v["frame_start"], v["frame_end"] + 1)
            step = max(1, len(span) // 12)
            rows = [r for f in span[::step] for r in session.guesses.get(f, []) if r["obj"] == v["obj"]]
            seen_cls = collections.Counter(r["cls"] for r in rows).most_common(1)
            seen_cls = seen_cls[0][0] if seen_cls else None
            if h.get("cls") in session.classes:
                if seen_cls != h["cls"]:
                    skipped.append("%s: the object sitting there (%s) is not a %s" % (h["name"], seen_cls or "unknown", h["cls"]))
                    continue
                cls = h["cls"]
            else:
                cls = seen_cls
            if cls is None:
                continue
            name = None
            names = collections.Counter()
            for f in span[::step]:
                nm = session.names_at(f).get(v["obj"])
                if nm:
                    names[nm] += 1
            if names:
                name = names.most_common(1)[0][0]
            if h.get("owner") and name is not None and name != h["owner"]:
                skipped.append("%s: the object sitting there is %s, not %s" % (h["name"], name, h["owner"]))
                continue
            for f in span:
                dets.setdefault(f, []).append(dict(x=h["x"], y=h["y"], cls=cls, score=HOME_SCORE, source="home",
                                                   heading_deg=h.get("facing_deg"), replace_px=float(h["radius"]),
                                                   name=name))
            out.append(dict(home=h["name"], obj=v["obj"], frame_start=v["frame_start"], frame_end=v["frame_end"],
                            seconds=round(v["still_s"], 1), name=name))
        note = ("Home positions: %d visit%s found, each a free guess of the visitor's place%s (shown for you to "
                "check; never accepted without you)." % (
                    len(out), "" if len(out) == 1 else "s",
                    " and facing while it sits there" if any(h.get("facing_deg") is not None for h in hl) else "")
                if out else "Home positions: no visit found in this recording yet.")
        if skipped:
            note += " Not used: %s." % "; ".join(skipped[:3]) + (" and %d more" % (len(skipped) - 3) if len(skipped) > 3 else "")
        return dict(dets=dets, visits=out, skipped=skipped, note=note)
    return refine
