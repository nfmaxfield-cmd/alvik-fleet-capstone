#!/usr/bin/env python3
"""Site profiles (conference-table build, Nick 2026-11-02): everything that differs between the home lab and a school
room, chosen by one word instead of edits in the code. Home stays the default and is exactly what ran before.

A site (tools/fleet/sites/<name>.json) says:
  robots        each robot's name (the name saved on the robot with the console's `name` command) and, optionally,
                its fixed WiFi address; robots without an address are found on the WiFi by their own hello (broadcast)
  pi            the Pi's addresses to show on screens (its Tailscale and WiFi names)
  conf_dir      where this site's calibrations, camera names, stored driving corrections and day records live
                (home: ~/.config/mirlt, as before; a school: ~/.config/mirlt/sites/<name>), so nothing measured at
                school ever overwrites home's and the reverse
  mc_dir        Mission Control's folder (workspace, production points, programs, runs)
  cameras       each camera role (the recorder's camera 0 is "overhead", camera 1 "side") with a label for the screens
  capture       the recorder's picture size and rate
  tracking      "lights" (home: robots' lights and outlines, cam_bridge.py) or "tags" (AprilTags on the roofs,
                tag_bridge.py)
  tags          family, printed size, roof size and height, each robot's tag id, spares
  table         the table's size when it is known (typed: a check on the measured one), else null
  (2026-11-03, school sticker points)
  points        {"use": "typed" | "clicked", "file": "<site>-points.csv", "overrides": [...]}: production points typed
                (Jay's sheet, sticker_points.py) or clicked in the pictures (the old way, the default)
  edge_in_cm    a point's robot centre at least this far from an open edge (default 17, home's)
  homes         {"use": "points" | "middle", "<robot>": "<point>", "face_deg": 90}: homes at named points, or the
                old homes near the table's middle (the default)
  keepouts      [{"name", "centre_in", "size_in", "solid", "use", "note"}]: the site's own keep-out rectangles
  production    {"zones": {"<robot>": [zones]}, "orders": null | {...}, "dwell_s": 3}: the default production mission
  run_tracking  "tags" | "lights" | "none": how the robots are followed during a run (default: the site's tracking).
                none = odometry only (each robot placed on its home by hand)

Which site is used: MIRLT_SITE in the environment, else the one saved by `sites.py use <name>` (the file
~/.config/mirlt/site), else home.

    python3 tools/fleet/sites.py                 # the site in use, in words
    python3 tools/fleet/sites.py list            # every site
    python3 tools/fleet/sites.py use school-conference
    python3 tools/fleet/sites.py get robot_args  # e.g. "--robot 192.168.1.250 ..." (for the launchers)
    python3 tools/fleet/sites.py env             # export lines for the launchers (MIRLT_CONF_DIR, ...)

(e24, Nick 24 Nov) Choices that belong to ONE computer (camera 2 on or off, a capture rate set by hand) live in
<conf dir>/site_local.json, laid over the site file, so a Pi never differs from GitHub:
    python3 tools/fleet/sites.py local                             # what is set on this computer
    python3 tools/fleet/sites.py local set cameras.side.use false
    python3 tools/fleet/sites.py local unset capture.fps
The capture rate (capture.fps) is the START value where the site has capture.fps_auto: after real runs it follows the
Pi's measured load (tools/pi/capture_rate.py).

No WiFi password is ever stored here or anywhere in this repo: the robots' networks are typed by Nick into the
firmware's secrets.h (git-ignored) on his laptop.
"""
import copy
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SITES_DIR = os.path.join(HERE, "sites")
BASE_CONF = "~/.config/mirlt"
DEFAULT_SITE = "home"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,10}$")    # the firmware keeps at most 10 characters (net.h setName)

# The keep-going policy (2026-11-03; Nick 2 Nov 21:01: "finishing the run with the highest confidence positional
# assessment it can make every step of the way"). Per site; home keeps exactly what it did before ("stop"):
#   on_problem         "keep_going": a problem that used to end the whole run (camera and pose disagree for long, out
#                      of re-plans, no plan, a robot lost) is said once, logged and worked round; only a full card,
#                      the operator's Stop and every robot out end it. "stop": as before
#   feel_out_trust     "camera": the feel-out reports a felt-vs-camera difference per side and logs it, but the cameras'
#                      outline stands and the production run never waits for it (AprilTag sites). "sensor": as before
#                      (a side moved in up to 10 cm, held past 10)
#   lost_robot_points  with keep_going: "reassign" (a robot that is out hands its remaining production points to the
#                      robots still running) or "skip" (they are listed as not done, as before)
POLICY_WORDS = {"on_problem": ("keep_going", "stop"), "feel_out_trust": ("camera", "sensor"),
                "lost_robot_points": ("reassign", "skip")}
POLICY_DEFAULTS = {"on_problem": "keep_going", "feel_out_trust": "sensor", "lost_robot_points": "reassign"}
# (e08, Nick 7 Nov Q7) keep going by default everywhere ("no stops ... rather a warning and a logged failure", 30 Oct;
# "stop only when safety can't be kept", 5 Nov); "stop" stays as the switch (a site's on_problem, run_plan --on-problem
# stop, mission_run --on-problem stop, the Run screen's tick-box, doe_loop --no-keep-going). Safety holds still happen

RUN_TRACKING_WORDS = {"tags": "roof AprilTags (tag bridge)",
                      "lights": "the robots' lights and outlines (camera bridge; corners clicked)",
                      "none": "no camera: odometry only (robots placed on their homes by hand; +- grows 0.07 cm per cm)"}

ROBOT_COLOURS = ["#2952CC", "#C2410C", "#7A3EC8", "#12805C", "#B54708", "#0E7490"]


def base_conf_dir():
    return os.path.expanduser(BASE_CONF)


def chosen_file():
    return os.path.join(base_conf_dir(), "site")


def local_dir():
    """(e07) sites made on this computer (a new area found by site_detect): kept out of the repo."""
    return os.path.join(base_conf_dir(), "sites-local")


def names():
    out = []
    for f in sorted(os.listdir(SITES_DIR)):
        if f.endswith(".json"):
            out.append(f[:-5])
    try:
        for f in sorted(os.listdir(local_dir())):
            if f.endswith(".json") and f[:-5] not in out:
                out.append(f[:-5])
    except OSError:
        pass
    return out


def current_name():
    env = os.environ.get("MIRLT_SITE", "").strip()
    if env:
        return env
    try:
        with open(chosen_file()) as f:
            v = f.read().strip()
        if v:
            return v
    except OSError:
        pass
    return DEFAULT_SITE


class SiteFileError(SystemExit):
    """A site file that cannot be used (risk review 2 Nov, #9: a typo silently turned the school into home). Raised by
    load(): the message names the file, the line and what is wrong, and every launcher stops on it. A SystemExit, so a
    command line prints the message alone (no traceback) and exits 1; programs that used to fall back to home's ways
    on a missing site re-raise this one (is_site_file_error)."""
    site_file_error = True


class SiteValueError(SiteFileError, ValueError):
    """A value in a site file that cannot be used (a name, an address, a tag id): a SiteFileError, and a ValueError as
    before 2026-11-03 (callers that caught ValueError still do)."""


def is_site_file_error(e):
    return bool(getattr(e, "site_file_error", False))


def _json_error_words(path, text, e):
    """'line 14, column 26: Expecting ',' delimiter' with the line itself and a caret under the spot."""
    lines = text.splitlines()
    ln = lines[e.lineno - 1] if 0 < e.lineno <= len(lines) else ""
    hint = ""
    if re.search(r"\d,\d", ln):
        hint = " (a comma inside a number? JSON numbers use a point: 6.2, not 6,2)"
    elif e.msg.startswith("Expecting property name") or "trailing" in e.msg:
        hint = " (a comma after the last item of a list or {...}? JSON allows none there)"
    elif e.msg.startswith("Expecting ',' delimiter"):
        hint = " (a missing comma between two items, or a quote missing round a word)"
    elif e.msg.startswith("Expecting value"):
        hint = " (a word without quotes? text goes in double quotes; empty values are null)"
    return ("The site file %s is broken at line %d, column %d: %s%s.\n  %4d | %s\n         %s^\n"
            "Nothing was started. Fix that line (on GitHub: open the file, the pencil, fix, Commit; then on the Pi: "
            "git pull), or switch site: ./run-fleet-pi.sh site home" % (
                os.path.relpath(path, REPO), e.lineno, e.colno, e.msg, hint, e.lineno, ln, " " * max(0, e.colno - 1)))


def _type_problems(d):
    """Values of the wrong kind (a number typed as text, a list where a number goes): [words]."""
    probs = []

    def num(v, where, null_ok=False):
        if v is None and null_ok:
            return
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            probs.append("%s should be a number%s, not %s" % (where, " or null" if null_ok else "", json.dumps(v)))
    if not isinstance(d, dict):
        return ["the file should be one {...} object"]
    if not isinstance(d.get("robots", []), list):
        probs.append('"robots" should be a list [...]')
    cap = d.get("capture")
    if cap is not None:
        if not isinstance(cap, dict):
            probs.append('"capture" should be {"width": ..., "height": ..., "fps": ...}')
        else:
            for k in ("width", "height", "fps"):
                if k in cap:
                    num(cap[k], "capture.%s" % k)
    tags = d.get("tags")
    if tags is not None:
        if not isinstance(tags, dict):
            probs.append('"tags" should be {...}')
        else:
            for k in ("tag_cm", "roof_cm"):
                if k in tags:
                    num(tags[k], "tags.%s" % k)
            if "height_cm" in tags:
                num(tags["height_cm"], "tags.height_cm", null_ok=True)
            if not isinstance(tags.get("ids", {}), dict):
                probs.append('tags.ids should be {"Alpha": 0, ...}')
    tb = d.get("table")
    if tb is not None:
        if not isinstance(tb, dict):
            probs.append('"table" should be {"length_cm": ..., "width_cm": ...} or null')
        else:
            for k in ("length_cm", "width_cm", "drop_cm"):
                if k in tb:
                    num(tb[k], "table.%s" % k, null_ok=True)
    if d.get("run_tracking") not in (None, "lights", "tags", "none"):
        probs.append('run_tracking should be "tags", "lights" or "none", not %s' % json.dumps(d.get("run_tracking")))
    pt = d.get("points")
    if pt is not None and (not isinstance(pt, dict) or pt.get("use", "clicked") not in ("typed", "clicked")):
        probs.append('"points" should be {"use": "typed" or "clicked", "file": ...}')
    if "edge_in_cm" in d:
        num(d["edge_in_cm"], "edge_in_cm")
    hm = d.get("homes")
    if hm is not None and (not isinstance(hm, dict) or hm.get("use", "middle") not in ("points", "middle")):
        probs.append('"homes" should be {"use": "points" or "middle", ...}')
    if d.get("keepouts") is not None and not isinstance(d.get("keepouts"), list):
        probs.append('"keepouts" should be a list [{...}]')
    if d.get("tracking") not in (None, "lights", "tags"):
        probs.append('tracking should be "lights" or "tags", not %s' % json.dumps(d.get("tracking")))
    rt = d.get("ref_tags")
    if rt is not None and not isinstance(rt, dict):
        probs.append('"ref_tags" should be {...}')
    # the keep-going policy (2026-11-03, Nick 2 Nov 21:01): each a word from its list
    for k, ok in POLICY_WORDS.items():
        if k in d and d[k] not in ok:
            probs.append('%s should be %s, not %s' % (k, " or ".join('"%s"' % w for w in ok), json.dumps(d[k])))
    return probs


def _check(d, name):
    probs = []
    seen_ids = {}
    for r in d.get("robots", []):
        if not NAME_RE.match(str(r.get("name", ""))):
            probs.append("robot name %r: 1-10 letters, digits, - or _ (the firmware keeps 10)" % r.get("name"))
        ip = r.get("ip")
        if ip and not re.match(r"^\d{1,3}(\.\d{1,3}){3}$", str(ip)):
            probs.append("robot %s: address %r is not like 192.168.1.40" % (r.get("name"), ip))
    tags = d.get("tags") or {}
    for n, i in (tags.get("ids") or {}).items():
        if i in seen_ids.values():
            probs.append("tag id %s used twice" % i)
        seen_ids[n] = i
    rt = d.get("ref_tags") or {}
    if rt:
        rids = [int(v) for v in list((rt.get("corners") or {}).values()) + list((rt.get("sides") or {}).values())]
        robot_ids = set(int(v) for v in (tags.get("ids") or {}).values()) | set(int(v) for v in tags.get("spares") or [])
        if len(set(rids)) != len(rids):
            probs.append("a reference tag id is used twice (ref_tags)")
        if set(rids) & robot_ids:
            probs.append("reference tag id(s) %s are also robot tags or spares (tags.ids / tags.spares)"
                         % ", ".join(str(i) for i in sorted(set(rids) & robot_ids)))
    if d.get("tracking") == "tags":
        for r in d.get("robots", []):
            if r.get("name") not in (tags.get("ids") or {}):
                probs.append("robot %s has no tag id (tags.ids)" % r.get("name"))
    if probs:
        raise SiteValueError("The site file tools/fleet/sites/%s.json has a value that cannot be used: %s. Nothing was "
                            "started. Fix it, or switch site: ./run-fleet-pi.sh site home" % (name, "; ".join(probs)))


def load(name=None):
    """The site as a dict, with defaults filled in and paths expanded (conf_dir, mc_dir absolute)."""
    name = name or current_name()
    path = os.path.join(SITES_DIR, name + ".json")
    if not os.path.exists(path) and os.path.exists(os.path.join(local_dir(), name + ".json")):
        path = os.path.join(local_dir(), name + ".json")     # (e07) a new area found on this computer
    if not os.path.exists(path):
        raise SiteFileError("No site called %r (sites: %s)%s. Nothing was started. Switch site: ./run-fleet-pi.sh site "
                            "home, or make one: copy tools/fleet/sites/school-conference.json to tools/fleet/sites/%s.json "
                            "and edit it." % (name, ", ".join(names()), " (chosen in %s)" % chosen_file()
                                              if not os.environ.get("MIRLT_SITE") and name != DEFAULT_SITE else
                                              " (MIRLT_SITE)" if os.environ.get("MIRLT_SITE") else "", name))
    with open(path) as f:
        text = f.read()
    try:
        d = json.loads(text)
    except ValueError as e:
        if hasattr(e, "lineno"):
            raise SiteFileError(_json_error_words(path, text, e))
        raise SiteFileError("The site file %s cannot be read: %s. Nothing was started." % (path, e))
    tp = _type_problems(d)
    if tp:
        raise SiteFileError("The site file tools/fleet/sites/%s.json has values of the wrong kind: %s. Nothing was "
                            "started. Fix it, or switch site: ./run-fleet-pi.sh site home" % (name, "; ".join(tp)))
    d = copy.deepcopy(d)
    d["name"] = name
    d.setdefault("title", name)
    d.setdefault("tracking", "lights")
    d.setdefault("robots", [])
    d.setdefault("pi", {})
    d.setdefault("cameras", {"overhead": {"label": "overhead camera"}, "side": {"label": "side camera"}})
    d.setdefault("capture", {"width": 640, "height": 480, "fps": 25})
    d.setdefault("table", None)
    d.setdefault("run_tracking", d["tracking"])
    for k, v in POLICY_DEFAULTS.items():
        d.setdefault(k, v)
    cd = d.get("conf_dir") or (BASE_CONF if name == DEFAULT_SITE else BASE_CONF + "/sites/" + name)
    d["conf_dir"] = os.path.expanduser(cd)
    apply_overrides(d)
    apply_local(d)          # (e24) this computer's own choices (camera 2 on or off, a capture rate set by hand)
    md = d.get("mc_dir") or ("logs/mission-control" if name == DEFAULT_SITE else "logs/mission-control-" + name)
    d["mc_dir"] = md if os.path.isabs(md) else os.path.join(REPO, md)
    for k, r in enumerate(d["robots"]):
        r.setdefault("colour", ROBOT_COLOURS[k % len(ROBOT_COLOURS)])
    _check(d, name)
    return d


OVERRIDES_FILE = "site_overrides.json"
OVERRIDE_KEYS = {"table": dict, "measure_trust": str, "corner_clicks": bool, "edge_in_cm": float,
                 "tags.height_cm": float, "calibration.min_firmware": str}


def overrides_path(site):
    return os.path.join(site["conf_dir"], OVERRIDES_FILE)


def apply_overrides(d):
    """(e07) Values changed in Mission Control's Advanced section (typed values, switches), kept on this computer in
    the site's folder (never in the repo), laid over the site file. {"tags.height_cm": 6.3, "table": {...}, ...}."""
    try:
        with open(overrides_path(d)) as f:
            ov = json.load(f)
    except (OSError, ValueError):
        return d
    for k, v in ov.items():
        if k not in OVERRIDE_KEYS:
            continue
        if "." in k:
            a, b = k.split(".", 1)
            d.setdefault(a, {})
            if isinstance(d[a], dict):
                if v is None:
                    d[a].pop(b, None)
                else:
                    d[a][b] = v
        elif v is None:
            d.pop(k, None) if k != "table" else d.__setitem__("table", None)
        else:
            d[k] = v
    d["_overrides"] = dict(ov)
    return d


# -- (e24, Nick 24 Nov D5) per-Pi choices, out of the tracked site file -------------------------------------------------
# On 18 and 23 Nov Jay-Sun changed the school's site file ON THE PI (camera 2 on, capture 15 -> 10): the Pi's copy then
# differed from GitHub and every update needed a hand merge. Choices that belong to ONE computer now live beside its
# camera names, in <conf dir>/site_local.json, laid over the repo's site file (which stays the default):
#     ./run-fleet-pi.sh local                               # what is set on this Pi
#     ./run-fleet-pi.sh local set cameras.side.use false    # camera 2 off on this Pi
#     ./run-fleet-pi.sh local set capture.fps 15            # the capture rate by hand (else decided from the load)
#     ./run-fleet-pi.sh local unset capture.fps             # back to the repo's value / the automatic rate
LOCAL_FILE = "site_local.json"
LOCAL_KEYS = {"cameras.overhead.use": bool, "cameras.side.use": bool, "capture.fps": float, "capture.width": int,
              "capture.height": int}


def local_path(site):
    return os.path.join(site["conf_dir"], LOCAL_FILE)


def read_local(site):
    try:
        with open(local_path(site)) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _local_value(key, value):
    """The value in the key's own kind, or ValueError in plain words."""
    kind = LOCAL_KEYS[key]
    if kind is bool:
        if isinstance(value, bool):
            return value
        v = str(value).strip().lower()
        if v in ("1", "true", "on", "yes"):
            return True
        if v in ("0", "false", "off", "no"):
            return False
        raise ValueError("%s takes true or false, not %r" % (key, value))
    try:
        v = kind(float(value))
    except (TypeError, ValueError):
        raise ValueError("%s takes a number, not %r" % (key, value))
    if v <= 0:
        raise ValueError("%s must be over 0" % key)
    return v


def apply_local(d):
    """Lay this computer's site_local.json over the site. Keys not in LOCAL_KEYS, or of the wrong kind, are left out
    and named in d["_local_refused"] (the launchers say them; nothing is guessed)."""
    used, refused = {}, {}
    for k, v in read_local(d).items():
        if k not in LOCAL_KEYS:
            refused[k] = "not a per-Pi setting (allowed: %s)" % ", ".join(sorted(LOCAL_KEYS))
            continue
        try:
            v = _local_value(k, v)
        except ValueError as e:
            refused[k] = str(e)
            continue
        node = d
        parts = k.split(".")
        for a in parts[:-1]:
            if not isinstance(node.get(a), dict):
                node[a] = {}
            node = node[a]
        node[parts[-1]] = int(v) if k == "capture.fps" and float(v).is_integer() else v
        used[k] = v
    if used:
        d["_local"] = used
    if refused:
        d["_local_refused"] = refused
    return d


def set_local(site, key, value):
    """Save one per-Pi choice (None removes it: the repo's value again). Returns the file's contents."""
    if key not in LOCAL_KEYS:
        raise ValueError("not a per-Pi setting: %s (allowed: %s)" % (key, ", ".join(sorted(LOCAL_KEYS))))
    cur = read_local(site)
    if value is None:
        cur.pop(key, None)
    else:
        cur[key] = _local_value(key, value)
    os.makedirs(site["conf_dir"], exist_ok=True)
    with open(local_path(site), "w") as f:
        json.dump(cur, f, indent=1)
    return cur


def local_from_edits(site_name, edited_text, repo_text):
    """(for `./run-fleet-pi.sh update`) The differences between a site file edited on this computer and the repo's
    version of it, as per-Pi choices: {key: value} for the LOCAL_KEYS that differ, and a list of other differences
    (kept in the update's backup, said to the person, not applied)."""
    try:
        mine, theirs = json.loads(edited_text), json.loads(repo_text)
    except ValueError as e:
        return {}, ["the edited file is not valid JSON (%s): kept in the backup only" % e]
    out, other = {}, []

    def get(d, key):
        for a in key.split("."):
            if not isinstance(d, dict) or a not in d:
                return None
            d = d[a]
        return d
    for k in LOCAL_KEYS:
        a, b = get(mine, k), get(theirs, k)
        if a is not None and a != b:
            out[k] = a

    def walk(a, b, path):
        if isinstance(a, dict) and isinstance(b, dict):
            for kk in sorted(set(a) | set(b)):
                walk(a.get(kk), b.get(kk), path + [kk])
        elif a != b:
            key = ".".join(path)
            if key not in LOCAL_KEYS and not key.endswith(".note") and not key.startswith("capture.fps_auto"):
                other.append("%s: this Pi had %s, the repo has %s" % (key, json.dumps(a)[:80], json.dumps(b)[:80]))
    walk(mine, theirs, [])
    return out, other


def capture_fps(site):
    """(e24) The recorder's capture rate in force and why: a rate set by hand on this Pi, else decided from the Pi's
    measured load (tools/pi/capture_rate.py) when the site file has capture.fps_auto, else the site file's."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
        import capture_rate as CRT
        return CRT.in_force(site)
    except Exception as e:  # noqa: BLE001  (the launchers always get a rate)
        return float((site.get("capture") or {}).get("fps", 25)), "the site file's (%s)" % e


def set_override(site, key, value):
    """Save one Advanced value for this site on this computer (None removes it: the site file's value again)."""
    if key not in OVERRIDE_KEYS:
        raise ValueError("not a setting Advanced can change: %s" % key)
    p = overrides_path(site)
    try:
        with open(p) as f:
            ov = json.load(f)
    except (OSError, ValueError):
        ov = {}
    if value is None or value == "":
        ov.pop(key, None)
    else:
        ov[key] = value
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w") as f:
        json.dump(ov, f, indent=1)
    return ov


def use(name):
    load(name)       # it must exist and be valid
    os.makedirs(base_conf_dir(), exist_ok=True)
    with open(chosen_file(), "w") as f:
        f.write(name + "\n")
    return name


def robot_names(site=None):
    s = site or load()
    return [r["name"] for r in s["robots"]]


def robot_args(site=None):
    """'--robot IP ...' for the robots with a fixed address ('' when none: they are found by their hello)."""
    s = site or load()
    return " ".join("--robot %s" % r["ip"] for r in s["robots"] if r.get("ip"))


def tag_ids(site=None):
    s = site or load()
    return dict((s.get("tags") or {}).get("ids") or {})


def colours(site=None):
    s = site or load()
    return {r["name"]: r["colour"] for r in s["robots"]}


def env_lines(site=None):
    """Shell export lines the launchers eval: the site's folders and robots."""
    s = site or load()
    out = ["export MIRLT_SITE=%s" % s["name"], "export MIRLT_CONF_DIR=%s" % _q(s["conf_dir"]),
           "export MIRLT_MC_DIR=%s" % _q(s["mc_dir"]), "export MIRLT_TRACKING=%s" % s["tracking"],
           "export MIRLT_ROBOTS=%s" % _q(",".join(robot_names(s))),
           "export MIRLT_ROBOT_ARGS=%s" % _q(robot_args(s)),
           "export MIRLT_CAPTURE=%s" % _q("--width %d --height %d" % (s["capture"]["width"], s["capture"]["height"])),
           "export MIRLT_CAPTURE_FPS=%d" % int(capture_fps(s)[0]),
           "export MIRLT_PI_ADDR=%s" % _q(s["pi"].get("address", ""))]
    return out


def _q(v):
    v = str(v)
    return "'" + v.replace("'", "'\"'\"'") + "'"


def words(site=None):
    s = site or load()
    rb = ", ".join("%s%s" % (r["name"], " at " + r["ip"] if r.get("ip") else " (found on the WiFi)") for r in s["robots"])
    lines = ["Site: %s (%s)" % (s["name"], s["title"]),
             "  robots: %s" % (rb or "none listed"),
             "  tracking: %s" % ("AprilTags on the roofs" if s["tracking"] == "tags" else "the robots' lights and outlines"),
             "  cameras: %s" % "; ".join("%s = %s" % (k, v.get("label", k)) for k, v in s["cameras"].items()),
             "  pictures: %d x %d at %d a second (%s)" % ((s["capture"]["width"], s["capture"]["height"]) +
                                                         (int(capture_fps(s)[0]), capture_fps(s)[1])),
             "  calibrations and camera names: %s" % s["conf_dir"],
             "  Mission Control's folder: %s" % os.path.relpath(s["mc_dir"], REPO)]
    if s.get("_local"):
        lines.append("  set on this computer only (%s): %s" % (local_path(s), ", ".join(
            "%s = %s" % (k, json.dumps(v)) for k, v in sorted(s["_local"].items()))))
    for k, why in sorted((s.get("_local_refused") or {}).items()):
        lines.append("  NOT USED from %s: %s: %s" % (LOCAL_FILE, k, why))
    if s.get("table"):
        lines.append("  table (typed): %s x %s cm" % (s["table"].get("length_cm"), s["table"].get("width_cm")))
    if s.get("run_tracking", s["tracking"]) != s["tracking"] or s.get("points"):
        lines.append("  during a run: %s" % RUN_TRACKING_WORDS.get(s.get("run_tracking"), s.get("run_tracking")))
    if (s.get("points") or {}).get("use") == "typed":
        lines.append("  production points: typed (%s), %d correction(s) to confirm; edge room %s cm" % (
            s["points"].get("file"), len(s["points"].get("overrides") or []), s.get("edge_in_cm", 17)))
    lines.append("  on a problem: %s" % ("keep going (said once, logged, worked round; stops only for a full card, "
                                          "Stop, or every robot out); a lost robot's points %s" % (
                                              "go to the robots still running" if s.get("lost_robot_points") ==
                                              "reassign" else "are listed as not done")
                                          if s.get("on_problem") == "keep_going" else "stop the run (as before)"))
    if s["tracking"] == "tags":
        lines.append("  feel-out: %s" % ("the cameras' outline is trusted; a felt difference is reported, never moves "
                                         "it or holds the run" if s.get("feel_out_trust") == "camera" else
                                         "a side felt 2-10 cm in is moved in; further holds the run"))
        t = s.get("tags") or {}
        lines.append("  tags: %s, %.1f cm black square on a %.1f cm roof, %s cm above the table; ids %s, spares %s" % (
            t.get("family"), t.get("tag_cm", 0), t.get("roof_cm", 0),
            t.get("height_cm") if t.get("height_cm") is not None else "NOT MEASURED YET",
            ", ".join("%s=%s" % kv for kv in (t.get("ids") or {}).items()), t.get("spares")))
    return "\n".join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cmd = argv[0] if argv else "show"
    if cmd == "show":
        print(words())
    elif cmd == "list":
        cur = current_name()
        for n in names():
            try:
                s = load(n)
                print("%s %-20s %s" % ("*" if n == cur else " ", n, s["title"]))
            except SiteFileError as e:
                print("%s %-20s BROKEN: %s" % ("*" if n == cur else " ", n, str(e).splitlines()[0]))
    elif cmd == "use":
        if len(argv) < 2:
            raise SystemExit("Which site? python3 tools/fleet/sites.py use <name> (sites: %s)" % ", ".join(names()))
        use(argv[1])
        print("Site now: %s. Every launcher and Mission Control opened from now on uses it." % argv[1])
        print(words(load(argv[1])))
    elif cmd == "get":
        key = argv[1] if len(argv) > 1 else ""
        s = load()
        if key == "robot_args":
            print(robot_args(s))
        elif key == "robots":
            print(",".join(robot_names(s)))
        elif key in ("conf_dir", "mc_dir", "tracking", "name"):
            print(s[key])
        else:
            print(json.dumps(s.get(key), indent=1))
    elif cmd == "env":
        print("\n".join(env_lines()))
    elif cmd == "local":
        # (e24) this computer's own choices for the site in use (never in the repo)
        s = load()
        sub = argv[1] if len(argv) > 1 else "show"
        try:
            if sub == "set" and len(argv) >= 4:
                set_local(s, argv[2], argv[3])
                print("Saved on this computer: %s = %s (%s). It applies from the next start of Mission Control or "
                      "the recorder." % (argv[2], argv[3], local_path(s)))
            elif sub == "unset" and len(argv) >= 3:
                set_local(s, argv[2], None)
                print("Removed: %s is the repo's value again (%s)." % (argv[2], local_path(s)))
            elif sub == "from-edits" and len(argv) >= 4:
                with open(argv[2]) as f1, open(argv[3]) as f2:
                    got, other = local_from_edits(s["name"], f1.read(), f2.read())
                for k, v in sorted(got.items()):
                    set_local(s, k, v)
                    print("KEPT on this computer: %s = %s" % (k, json.dumps(v)))
                for w in other:
                    print("NOT CARRIED OVER (it is in the backup): %s" % w)
                if not got and not other:
                    print("No per-Pi difference in the edited site file.")
            elif sub != "show":
                raise ValueError("usage: local | local set <key> <value> | local unset <key>   (keys: %s)"
                                 % ", ".join(sorted(LOCAL_KEYS)))
        except ValueError as e:
            raise SystemExit(str(e))
        s = load()
        loc = s.get("_local") or {}
        print("Set on this computer only for %s (%s):" % (s["name"], local_path(s)))
        print("\n".join("  %s = %s" % (k, json.dumps(v)) for k, v in sorted(loc.items())) or
              "  nothing: every value is the repo's site file's")
        for k, why in sorted((s.get("_local_refused") or {}).items()):
            print("  NOT USED: %s: %s" % (k, why))
        print("Camera 2 (side): %s. Capture rate: %g a second (%s)." % (
            "ON" if ((s.get("cameras") or {}).get("side") or {}).get("use", True) else "off",
            capture_fps(s)[0], capture_fps(s)[1]))
    else:
        raise SystemExit(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
