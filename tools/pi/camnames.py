#!/usr/bin/env python3
"""Cameras chosen by name, not by number (Nick, 2026-10-26: after a Pi restart the /dev/video numbers swapped and the
runner opened the side camera as the overhead).

A small file on the Pi, ~/.config/mirlt/cameras.json, says which camera name is which:
    {"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}
A name matches when it is part of the camera's own name (the one `v4l2-ctl --list-devices` and
`python3 tools/pi/camrec.py --list` print), ignoring upper/lower case. Each USB webcam makes two or more
/dev/video numbers; the one that sends pictures is the camera's first one (index 0 in /sys/class/video4linux).
If the file is missing: at home the two names above (the cameras on the Pi on 2026-10-26); at any other site its
own "camera_names" block in tools/fleet/sites/<site>.json (school: overhead "C920e", side off), never home's names
(5 Nov 2026). An existing cameras.json always wins.
A role can also name the USB port, for two cameras with the same name: {"overhead": {"name": "USB 2.0 Camera",
"port": "1.4"}} (the port is the last part of camrec's "(usb-0000:01:00.0-1.4)").

    python3 tools/pi/camnames.py                    # what is plugged in, and which is the overhead and the side
    python3 tools/pi/camnames.py --dev overhead     # prints the overhead's number (e.g. 2), or says why not and exits 1
    python3 tools/pi/camnames.py --set overhead "USB 2.0 Camera"     # change which name is the overhead

run-e02-pi.sh uses --dev; OVERHEAD_DEV / SIDE_DEV in front of it still pick a number by hand.
"""
import json
import os
import re
import sys

VERSION = "2026-11-10e14"   # e14 (school 18:12): a camera picked by the fallback (a name not found) must be a USB
                            # capture camera (real_camera: a USB port, and VIDIOC_QUERYCAP not saying codec / no capture);
                            # scan leaves out a node the kernel says is no capture device. 11-10 (7f041c8): the Pi's
                            # own bcm2835 / rpi- / pispbe / rp1-cfe / unicam nodes skipped by name. No version string before

def _site():
    """The site in use (tools/fleet/sites.py) or None (no site module)."""
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fleet"))
        import sites
        return sites.load()
    except BaseException as e:      # noqa: BLE001  (no site module: home's folder)
        if getattr(e, "site_file_error", False):
            raise                       # a broken or missing site file stops loudly (2026-11-03), never home's names
        return None


def _conf_dir():
    """MIRLT_CONF_DIR, else the site in use's folder (tools/fleet/sites.py, 2026-11-02: a school site keeps its own
    camera names; home's is ~/.config/mirlt as before)."""
    v = os.environ.get("MIRLT_CONF_DIR")
    if v:
        return os.path.expanduser(v)
    s = _site()
    return s["conf_dir"] if s else os.path.expanduser("~/.config/mirlt")


def site_defaults(site=None):
    """(5 Nov 2026, the school Pi: `rec` looked for home's "USB 2.0 Camera" because the school's folder had no
    cameras.json) The names to use when there is no cameras.json: home's DEFAULTS for the home site only. Any other
    site: its own "camera_names" block ({"overhead": "C920e", "side": ""}; "" or null = that camera not used), else
    NO names (resolve then says to run --set). Never home's names for another site. (dict, words)."""
    s = _site() if site is None else site
    name = (s or {}).get("name", "home")
    if name == "home":
        return dict(DEFAULTS), "the built-in home defaults"
    cn = (s or {}).get("camera_names") or {}
    out = {r: str(cn.get(r) or "") for r in ROLES}
    if any(out.values()):
        return out, "the site %s's camera_names (tools/fleet/sites/%s.json)" % (name, name)
    return out, "no names (the site %s lists no camera_names; home's are not used for another site)" % name


CONF_PATH = os.path.join(_conf_dir(), "cameras.json")
DEFAULTS = {"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}    # home's cameras (2026-10-26)
ROLES = ("overhead", "side")
SYSFS = os.environ.get("MIRLT_V4L_SYSFS", "/sys/class/video4linux")     # tests point it at a stand-in folder


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


NOT_CAMERAS = ("bcm2835", "rpi-", "pispbe", "rp1-cfe", "unicam")   # the Pi's built-in video devices (no picture)
DEV_DIR = os.environ.get("MIRLT_V4L_DEV", "/dev")      # (e14) where /dev/videoN live (tests: a folder with none)
VIDIOC_QUERYCAP = 0x80685600     # _IOR('V', 0, struct v4l2_capability): 104 bytes
CAP_VIDEO_CAPTURE, CAP_VIDEO_CAPTURE_MPLANE = 0x1, 0x1000
CAP_M2M, CAP_M2M_MPLANE, CAP_DEVICE_CAPS = 0x8000, 0x4000, 0x80000000


def capture_caps(dev, dev_dir=None):
    """(e14, school 18:12: /dev/video10, bcm2835-codec-decode, was picked as the side camera) True when /dev/videoN is a
    video CAPTURE device and not a memory-to-memory one (a codec, the ISP): the kernel's own answer (VIDIOC_QUERYCAP,
    the node's device_caps). None when it cannot be asked (no such node here, no permission: tests, a laptop)."""
    path = os.path.join(dev_dir or DEV_DIR, "video%d" % int(dev))
    try:
        import fcntl
        import struct
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except (OSError, ImportError):
        return None
    try:
        buf = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
        caps, dcaps = struct.unpack_from("<II", buf, 84)
    except OSError:
        return None
    finally:
        os.close(fd)
    c = dcaps if caps & CAP_DEVICE_CAPS else caps
    return bool(c & (CAP_VIDEO_CAPTURE | CAP_VIDEO_CAPTURE_MPLANE)) and not (c & (CAP_M2M | CAP_M2M_MPLANE))


def real_camera(c):
    """(e14) May a fallback pick this one as a camera: it hangs off a USB port, and the kernel does not say it is no
    capture device (capture False)."""
    return bool(c.get("usb", bool(c.get("port")))) and c.get("capture") is not False


def scan(sysfs=None):
    """Every camera the Pi sees: [dict(dev=N, name, port, nodes=[N, ...])], one per camera (its picture node first).
    The picture node is the one whose sysfs 'index' is 0 (the others only send information)."""
    sysfs = sysfs or SYSFS
    try:
        entries = os.listdir(sysfs)
    except OSError:
        return []
    by_cam = {}
    for e in entries:
        m = re.fullmatch(r"video(\d+)", e)
        if not m:
            continue
        n = int(m.group(1))
        base = os.path.join(sysfs, e)
        name = _read(os.path.join(base, "name"))
        idx = _read(os.path.join(base, "index"))
        idx = int(idx) if idx.isdigit() else 0
        # the USB interface the node hangs off, e.g. .../usb1/1-1/1-1.4/1-1.4:1.0 -> port "1.4"
        port = ""
        try:
            dev = os.path.realpath(os.path.join(base, "device"))
            iface = os.path.basename(dev)
            pm = re.fullmatch(r"\d+-([\d.]+):[\d.]+", iface)
            if pm:
                port = pm.group(1)
            else:
                port = _read(os.path.join(base, "port"))      # tests: a plain file
        except OSError:
            pass
        if not port and name.lower().startswith(NOT_CAMERAS):
            continue        # (11-10, school 18:12) the Pi's own codec / ISP / decoder nodes are not cameras
        by_cam.setdefault((name, port) if port else (name, "dev%d" % n), []).append((idx, n))
    cams = []
    for (name, port), nodes in by_cam.items():
        nodes.sort()
        pic = [n for i, n in nodes if i == 0]
        if not pic:
            continue                     # only information nodes
        cap = capture_caps(pic[0])
        if cap is False:
            continue                     # (e14) the kernel says it is no capture device (a codec, the ISP)
        cams.append(dict(dev=pic[0], name=name, port="" if port.startswith("dev") else port,
                         nodes=sorted(n for _, n in nodes), usb=bool(port) and not port.startswith("dev"),
                         capture=cap))
    cams.sort(key=lambda c: c["dev"])
    return cams


BY_ID = os.environ.get("MIRLT_V4L_BY_ID", "/dev/v4l/by-id")   # tests point it at a stand-in folder


def by_id_names(folder=None):
    """{video number: its stable name under /dev/v4l/by-id} (2026-11-03). The kernel's /dev/videoN numbers can swap
    after a restart; these names stay (they carry the camera's maker, model and serial, when it has one). The launchers
    already find each camera by its name and USB port at every start (resolve below), which is what by-id gives, so
    the numbers swapping never picks the wrong camera; these names are shown to check by eye."""
    folder = folder or BY_ID
    out = {}
    try:
        names = sorted(os.listdir(folder))
    except OSError:
        return out
    for n in names:
        m = re.fullmatch(r"video(\d+)", os.path.basename(os.path.realpath(os.path.join(folder, n))))
        if m and n.endswith("-index0"):
            out[int(m.group(1))] = n
    return out


def load_conf(path=None, defaults=None):
    """{role: dict(name, port)} from the file, or the defaults (with from_file False). An existing cameras.json always
    wins (Nick's camnames.py --set). defaults: None = site_defaults() when reading the site's own file, home's DEFAULTS
    for any other path (tests)."""
    if defaults is None:
        defaults = site_defaults()[0] if (path is None or path == CONF_PATH) else DEFAULTS
    path = path or CONF_PATH
    raw, from_file = dict(defaults), False
    try:
        with open(path) as f:
            d = json.load(f)
        if isinstance(d, dict):
            raw.update({k: v for k, v in d.items() if k in ROLES})
            from_file = True
    except (OSError, ValueError):
        pass
    out = {}
    for r in ROLES:
        v = raw.get(r)
        if isinstance(v, dict):
            out[r] = dict(name=str(v.get("name") or ""), port=str(v.get("port") or ""))
        else:
            out[r] = dict(name=str(v or ""), port="")
    return out, from_file


def save_conf(conf, path=None):
    path = path or CONF_PATH
    os.makedirs(os.path.dirname(path), exist_ok=True)
    d = {r: (dict(name=c["name"], port=c["port"]) if c.get("port") else c["name"]) for r, c in conf.items()}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def _matches(cam, want):
    if not want.get("name") or want["name"].lower() not in cam["name"].lower():
        return False
    return not want.get("port") or want["port"] == cam["port"]


def resolve(role, cams=None, conf=None):
    """(number or None, words). words says what was found, or plainly why not. (11-05h, Observe then trust) The names
    are a guess: when the named camera is not found, observe.pick_cameras decides from what is plugged in (one camera:
    the overhead, whatever its name; two: the names tell them apart, else the one not taken). conf["observe"] False
    switches this off (the old strict names)."""
    if cams is None:
        cams = scan()
    if conf is None:
        conf, _ = load_conf()
    dev, words = _resolve_named(role, cams, conf)
    if dev is not None or conf.get("observe") is False or not cams:
        return dev, words
    if role == "side" and not (conf.get("side") or {}).get("name"):
        return dev, words                    # camera 2 switched off ("" name): stays off
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "fleet"))
    import observe as OB
    want = {r: (conf.get(r) or {}).get("name", "") for r in ROLES}
    # (e14, school 18:12) a fallback picks only a real capture camera on a USB port: never the Pi's own video nodes
    real = [c for c in cams if real_camera(c)]
    if not real:
        return dev, words + " (and nothing else plugged in is a USB capture camera)"
    m, w = OB.pick_cameras([(c["dev"], c["name"]) for c in real], want)
    if role in m:
        c = next(c for c in cams if c["dev"] == m[role])
        return c["dev"], "%s camera: %s at /dev/video%d (picked from what is plugged in: %s)" % (
            role, c["name"], c["dev"], "; ".join(w) or "names did not match")
    return dev, words


def _resolve_named(role, cams, conf):
    want = conf.get(role) or dict(name="", port="")
    if not want["name"]:
        seen = "; ".join("%s at /dev/video%d" % (c["name"], c["dev"]) for c in cams) or "no cameras at all"
        return None, ("no camera name is set for the %s camera (%s; no file: the site's camera_names). The Pi sees: "
                      "%s. Name it: python3 tools/pi/camnames.py --set %s \"<part of its name>\"" % (
                          role, CONF_PATH, seen, role))
    hit = [c for c in cams if _matches(c, want)]
    label = want["name"] + (" on USB port %s" % want["port"] if want["port"] else "")
    if not hit:
        seen = "; ".join("%s at /dev/video%d" % (c["name"], c["dev"]) for c in cams) or "no cameras at all"
        return None, "the %s camera (%s) is not plugged in or not found. The Pi sees: %s" % (role, label, seen)
    if len(hit) > 1:
        return None, ("two cameras match the %s camera's name (%s: %s). Add its USB port to %s, e.g. "
                      '{"%s": {"name": "%s", "port": "%s"}}' % (
                          role, label, ", ".join("/dev/video%d port %s" % (c["dev"], c["port"] or "?") for c in hit),
                          CONF_PATH, role, want["name"], hit[0]["port"] or "1.4"))
    others = [r for r in ROLES if r != role and conf.get(r, {}).get("name")]
    for r in others:
        if _matches(hit[0], conf[r]):
            return None, "the %s and %s cameras have the same name in %s: fix one" % (role, r, CONF_PATH)
    c = hit[0]
    return c["dev"], "%s camera: %s at /dev/video%d" % (role, c["name"], c["dev"])


def report(cams=None, conf=None):
    """For the start page: dict(cameras=[...with role], roles={role: dict(dev, ok, words)})."""
    if cams is None:
        cams = scan()
    if conf is None:
        conf, _ = load_conf()
    roles = {}
    for r in ROLES:
        dev, words = resolve(r, cams, conf)
        roles[r] = dict(dev=dev, ok=dev is not None, words=words, name=conf[r]["name"])
    for c in cams:
        c["role"] = next((r for r in ROLES if roles[r]["dev"] == c["dev"]), "")
    return dict(cameras=cams, roles=roles)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["--dev"] and len(argv) == 2:
        dev, words = resolve(argv[1])
        if dev is None:
            print("Camera problem: " + words, file=sys.stderr)
            return 1
        print(dev)
        return 0
    if argv[:1] == ["--set"] and len(argv) in (3, 4) and argv[1] in ROLES:
        # (2026-11-02) a 4th word: the USB port, for two cameras with the same name (the school's two new cameras)
        conf, _ = load_conf()
        conf[argv[1]] = dict(name=argv[2], port=argv[3] if len(argv) == 4 else "")
        save_conf(conf)
        print("Saved: the %s camera is the one named '%s'%s (%s)." % (argv[1], argv[2], (" on USB port %s" % argv[3])
                                                                    if len(argv) == 4 else "", CONF_PATH))
        return 0
    if argv[:1] == ["--init"]:
        # the installer: write the file with today's names if there is none yet (never overwrites)
        if not os.path.exists(CONF_PATH):
            d, src = site_defaults()
            if not any(d.values()):
                print("No camera names to save: %s. Name the overhead camera: python3 tools/pi/camnames.py --set "
                      "overhead \"<part of its name>\"" % src)
                return 1
            save_conf(load_conf()[0])
            print("Camera names saved in %s from %s (overhead: %s; side: %s)." % (
                CONF_PATH, src, d["overhead"] or "none", d["side"] or "none, switched off"))
        else:
            print("Camera names already set in %s (kept)." % CONF_PATH)
        return 0
    if argv:
        print(__doc__)
        return 2
    conf, from_file = load_conf()
    rep = report(conf=conf)
    print("Camera names from %s" % (CONF_PATH if from_file else "%s (no %s yet)" % (site_defaults()[1], CONF_PATH)))
    byid = by_id_names()
    for c in rep["cameras"]:
        print("  /dev/video%-2d %-40s USB port %-6s %s" % (c["dev"], c["name"], c["port"] or "?",
                                                         ("<- " + c["role"].upper()) if c["role"] else ""))
        if c["dev"] in byid:
            print("               stable name: /dev/v4l/by-id/%s" % byid[c["dev"]])
    if not rep["cameras"]:
        print("  (no cameras found)")
    for r in ROLES:
        if r == "side" and not rep["roles"][r]["name"]:
            print("OFF: the side camera (camera 2) is switched off here (no name set): the overhead camera alone")
            continue
        print(("OK: " if rep["roles"][r]["ok"] else "MISSING: ") + rep["roles"][r]["words"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
