#!/usr/bin/env python3
"""Cameras chosen by name, not by number (Nick, 2026-10-26: after a Pi restart the /dev/video numbers swapped and the
runner opened the side camera as the overhead).

A small file on the Pi, ~/.config/mirlt/cameras.json, says which camera name is which:
    {"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}
A name matches when it is part of the camera's own name (the one `v4l2-ctl --list-devices` and
`python3 tools/pi/camrec.py --list` print), ignoring upper/lower case. Each USB webcam makes two or more
/dev/video numbers; the one that sends pictures is the camera's first one (index 0 in /sys/class/video4linux).
If the file is missing, the two names above are used (the cameras on the Pi on 2026-10-26).
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

CONF_PATH = os.path.join(os.path.expanduser(os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt")), "cameras.json")
DEFAULTS = {"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}
ROLES = ("overhead", "side")
SYSFS = os.environ.get("MIRLT_V4L_SYSFS", "/sys/class/video4linux")     # tests point it at a stand-in folder


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


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
        by_cam.setdefault((name, port) if port else (name, "dev%d" % n), []).append((idx, n))
    cams = []
    for (name, port), nodes in by_cam.items():
        nodes.sort()
        pic = [n for i, n in nodes if i == 0]
        if not pic:
            continue                     # only information nodes
        cams.append(dict(dev=pic[0], name=name, port="" if port.startswith("dev") else port,
                         nodes=sorted(n for _, n in nodes)))
    cams.sort(key=lambda c: c["dev"])
    return cams


def load_conf(path=None):
    """{role: dict(name, port)} from the file, or the defaults (with from_file False)."""
    path = path or CONF_PATH
    raw, from_file = dict(DEFAULTS), False
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
    """(number or None, words). words says what was found, or plainly why not."""
    if cams is None:
        cams = scan()
    if conf is None:
        conf, _ = load_conf()
    want = conf.get(role) or dict(name="", port="")
    if not want["name"]:
        return None, "no camera name is set for the %s camera (%s)" % (role, CONF_PATH)
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
    if argv[:1] == ["--set"] and len(argv) == 3 and argv[1] in ROLES:
        conf, _ = load_conf()
        conf[argv[1]] = dict(name=argv[2], port="")
        save_conf(conf)
        print("Saved: the %s camera is the one named '%s' (%s)." % (argv[1], argv[2], CONF_PATH))
        return 0
    if argv[:1] == ["--init"]:
        # the installer: write the file with today's names if there is none yet (never overwrites)
        if not os.path.exists(CONF_PATH):
            save_conf(load_conf()[0])
            print("Camera names saved in %s (overhead: %s; side: %s)." % (CONF_PATH, DEFAULTS["overhead"],
                                                                         DEFAULTS["side"]))
        else:
            print("Camera names already set in %s (kept)." % CONF_PATH)
        return 0
    if argv:
        print(__doc__)
        return 2
    conf, from_file = load_conf()
    rep = report(conf=conf)
    print("Camera names from %s" % (CONF_PATH if from_file else "the built-in defaults (no %s yet)" % CONF_PATH))
    for c in rep["cameras"]:
        print("  /dev/video%-2d %-40s USB port %-6s %s" % (c["dev"], c["name"], c["port"] or "?",
                                                         ("<- " + c["role"].upper()) if c["role"] else ""))
    if not rep["cameras"]:
        print("  (no cameras found)")
    for r in ROLES:
        print(("OK: " if rep["roles"][r]["ok"] else "MISSING: ") + rep["roles"][r]["words"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
