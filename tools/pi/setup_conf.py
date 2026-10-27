#!/usr/bin/env python3
"""What the camera setup page (tools/pi/camera_setup.py, 2026-10-27) saved on this Pi, for the launchers to read.

Two small files in ~/.config/mirlt (MIRLT_CONF_DIR overrides it; the same folder as camnames.py's cameras.json):
    overhead_calib          one line: the path of the overhead camera's floor calibration made by the setup page
    side_calib              one line: the side camera's floor calibration (two-camera tracking, 2026-10-27)
    overhead_exposure.json  {"exposure": 78, "saved": "...", "why": "..."}: the overhead camera's manual exposure
                            (V4L2 units of 100 us), chosen with the setup page's flash check
The calibration files themselves go in ~/.config/mirlt/calib/ (outside the repo, so the Pi's git pull never meets
them); the setup page also copies each one into its logs folder, which ./run-fleet-pi.sh stop uploads.

    python3 tools/pi/setup_conf.py calib        # prints the calibration path to use (roam1 if none saved, said on stderr)
    python3 tools/pi/setup_conf.py exposure     # prints the saved overhead exposure, or nothing (camera's automatic)
    python3 tools/pi/setup_conf.py side         # prints the side camera's calibration path, or nothing (overhead alone)
    python3 tools/pi/setup_conf.py show         # both, in words

No OpenCV import: the launchers call this at every start.
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
ROAM1 = os.path.join(REPO, "tools", "experiment", "data", "overhead_floorcal_roam1.json")
ROAM1_REL = "tools/experiment/data/overhead_floorcal_roam1.json"


def conf_dir():
    return os.path.expanduser(os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt"))


def _pointer():
    return os.path.join(conf_dir(), "overhead_calib")


def _expo_file():
    return os.path.join(conf_dir(), "overhead_exposure.json")


def overhead_calib(relative=True):
    """(path, words). The setup page's saved calibration if its pointer names a file that exists, else roam1, with
    words saying which and why (the caller prints them)."""
    fallback = ROAM1_REL if relative else ROAM1
    p = _pointer()
    try:
        with open(p) as f:
            path = f.read().strip()
    except OSError:
        return fallback, ("No calibration saved by the camera setup page (%s missing): using yesterday's roam1 "
                          "calibration %s. If the overhead camera has moved, run ./run-fleet-pi.sh setup first." % (p, ROAM1_REL))
    if not path or not os.path.exists(path):
        return fallback, ("The camera setup page's pointer (%s) names %s, which is not there: using roam1 (%s). Run "
                          "./run-fleet-pi.sh setup again." % (p, path or "nothing", ROAM1_REL))
    return path, "Overhead calibration from the camera setup page: %s" % path


def save_calib_pointer(path):
    os.makedirs(conf_dir(), exist_ok=True)
    tmp = _pointer() + ".tmp"
    with open(tmp, "w") as f:
        f.write(os.path.abspath(path) + "\n")
    os.replace(tmp, _pointer())
    return _pointer()


def calib_dir():
    return os.path.join(conf_dir(), "calib")


def new_calib_path(now=None):
    return os.path.join(calib_dir(), "overhead_floorcal_%s.json" % time.strftime("%Y%m%d-%H%M%S", time.localtime(now)))


def _side_pointer():
    return os.path.join(conf_dir(), "side_calib")


def side_calib():
    """(path or None, words): the side camera's floor calibration saved by the setup page (two-camera tracking,
    2026-10-27). None: the tracking runs on the overhead camera alone (and the robots' lights)."""
    p = _side_pointer()
    try:
        with open(p) as f:
            path = f.read().strip()
    except OSError:
        return None, ("No side camera calibration saved (%s missing): tracking with the overhead camera alone. To use "
                      "both cameras: ./run-fleet-pi.sh setup, part 2 (side camera)." % p)
    if not path or not os.path.exists(path):
        return None, "The side calibration pointer (%s) names %s, which is not there: overhead camera alone." % (
            p, path or "nothing")
    return path, "Side camera calibration from the camera setup page: %s" % path


def save_side_pointer(path):
    os.makedirs(conf_dir(), exist_ok=True)
    tmp = _side_pointer() + ".tmp"
    with open(tmp, "w") as f:
        f.write(os.path.abspath(path) + "\n")
    os.replace(tmp, _side_pointer())
    return _side_pointer()


def new_side_calib_path(now=None):
    return os.path.join(calib_dir(), "side_floorcal_%s.json" % time.strftime("%Y%m%d-%H%M%S", time.localtime(now)))


def overhead_exposure():
    """(value or None, the saved record or {}). A missing, unreadable or nonsense file = None (the camera's own)."""
    try:
        with open(_expo_file()) as f:
            d = json.load(f)
        v = float(d.get("exposure"))
    except (OSError, ValueError, TypeError, AttributeError):
        return None, {}
    if not (v > 0):
        return None, {}
    return v, d


def save_exposure(value, **info):
    """Save the overhead exposure (V4L2 units); value None removes it (back to the camera's automatic)."""
    p = _expo_file()
    if value is None:
        try:
            os.remove(p)
        except OSError:
            pass
        return p
    os.makedirs(conf_dir(), exist_ok=True)
    d = dict(exposure=round(float(value), 1), saved=time.strftime("%Y-%m-%d %H:%M:%S"), **info)
    with open(p + ".tmp", "w") as f:
        json.dump(d, f, indent=1)
    os.replace(p + ".tmp", p)
    return p


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    what = argv[0] if argv else "show"
    if what == "calib":
        path, words = overhead_calib()
        print(words, file=sys.stderr)
        print(path)
    elif what == "exposure":
        v, d = overhead_exposure()
        if v is not None:
            print("%g" % v)
            print("Overhead exposure from the camera setup page: %g (saved %s)" % (v, d.get("saved", "?")), file=sys.stderr)
    elif what == "side":
        path, words = side_calib()
        print(words, file=sys.stderr)
        if path:
            print(path)
    elif what == "show":
        print(overhead_calib()[1])
        print(side_calib()[1])
        v, d = overhead_exposure()
        print("Overhead exposure: %s" % ("%g (V4L2 units of 100 us; saved %s)" % (v, d.get("saved", "?")) if v is not None
                                          else "the camera's own (nothing saved)"))
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
