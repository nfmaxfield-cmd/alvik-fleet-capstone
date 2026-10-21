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
HOME_SCORE = 0.97            # a guess from a home visit: the place and facing are known exactly
NO_FLOOR = ("Outside data needs this camera's floor calibration (which pixel is which spot on the floor), and "
            "there is none yet. Make one as in tools/recognition/CROSSCAM.md: click about 8-30 floor points whose "
            "positions you measured (cm), fit the camera with cameras.py, and save it in the project as "
            "cameras/%s.json. Then come back here. Nothing was changed.")


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


def floor_camera(proj, rec):
    import outside
    for p in floor_paths(proj, rec):
        if os.path.exists(p):
            try:
                return outside.load_floor_map(p), p
            except Exception:
                continue
    return None, None


def no_floor_text(proj, rec):
    import project as projmod
    return NO_FLOOR % projmod.slug(proj.camera_key(rec))


# =============================================================================== settings per recording
def plugin_list():
    import outside
    return [dict(id=i, title=t, description=d) for i, t, d in outside.plugins()]


def plugin_words(plugin_id):
    """What the guesses are called on the page ("Guesses from motion and robot logs")."""
    return {"alvik": "robot logs", "csv": "the positions file"}.get(plugin_id, "outside data")


def settings_path(proj, rid):
    return os.path.join(proj.labels_dir(rid), SETTINGS_FILE)


def read_settings(proj, rid):
    return _read(settings_path(proj, rid))


def save_settings(proj, rid, path, plugin=None, offset_s=0.0):
    os.makedirs(proj.labels_dir(rid), exist_ok=True)
    s = dict(path=os.path.abspath(os.path.expanduser(path)), plugin=plugin or None, offset_s=float(offset_s or 0.0),
             added=time.strftime("%Y-%m-%dT%H:%M:%S"))
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


def read_data(proj, rec, path, plugin=None, offset_s=0.0):
    """Read the outside data with its plug-in (plain ValueError when it can't)."""
    import outside
    path = os.path.abspath(os.path.expanduser((path or "").strip().strip('"')))
    if not os.path.exists(path):
        raise ValueError("There is nothing at %s. Nothing was changed." % path)
    opts = dict(recording=rec["path"])
    if offset_s:
        opts["offset_s"] = float(offset_s)
    try:
        return outside.read_outside(path, plugin or None, **opts)
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
    cam, cam_path = floor_camera(proj, rec)
    if cam is None:
        raise ValueError(no_floor_text(proj, rec))
    times = _frame_times(rec)
    if not times:
        raise ValueError("This recording has no shared clock (frames.csv with t_wall), so outside data can't be lined "
                         "up with its frames. Recordings from the Pi have it; a video file made into a recording does "
                         "not. Nothing was changed.")
    say("Reading the outside data ...")
    data = read_data(proj, rec, st["path"], st.get("plugin"), st.get("offset_s"))
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
                             float(st.get("offset_s") or 0.0) + clock_found["offset_s"])
            names, _detail = outside.match_names(data.streams, tracks_cm)
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
    add(outside.anchors_from_labels(labelled, times, cam, names, heights))
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
               names={str(o): nm for o, nm in names.items()}, summary=summary,
               notes=list(data.notes) + ([clock_found["text"]] if clock_found else []),
               clock=data.clock, clock_found=clock_found, n=n, labelled_frames=len(labelled), frames=compact)
    _write(os.path.join(proj.labels_dir(rec["id"]), DRAFTS_FILE), out)
    return {k: v for k, v in out.items() if k != "frames"}


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
# Ready-made home positions from two clicks (the page shows these words; homes.py does the geometry).
HOME_PRESETS = [dict(id="alvik_dock", title="Alvik dock", name="Dock", owner_label="Robot that docks here",
                     owner_title="The robot's name in its log; blank: whichever robot sits there",
                     click1="Click the middle of the back wall's top inner edge (where a docked robot's nose touches).",
                     click2="Now click the middle of the funnel's mouth (the open end).",
                     added="the parked place and the dock's post stations")]


def home_preset(preset_id):
    for p in HOME_PRESETS:
        if p["id"] == preset_id:
            return p
    raise ValueError("No home-position preset %r." % preset_id)


def preset_homes(preset_id, name, camera_key, cam, click1, click2, owner=None, version=2, stations=True):
    """A preset's home positions from its two clicks (homes.py does the geometry)."""
    import homes as homesmod
    if preset_id == "alvik_dock":
        return homesmod.alvik_dock_homes(name, camera_key, cam, click1, click2, owner=owner, version=version,
                                         stations=stations)
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
