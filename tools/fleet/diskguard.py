#!/usr/bin/env python3
"""Free space on the Pi (2026-10-28): where frames go, and the guard every long run uses.

The DOE loop of 2026-10-27 21:34 died at 22:03 with "No space left on device": its frames went to logs/<session>/frames
on the Pi's SD card (the card), next to the logs. Now:
  - frames (and other big files) go to the USB drive the recorder uses (the first writable drive under /media/<user>),
    in <drive>/fleet-frames/<session>/frames; the session folder gets frames_where.txt saying where, so the upload
    (run-fleet-pi.sh doe-finish, mission_run --upload) finds them. No drive: logs/<session>/frames as before;
  - the guard: before a run starts and between its loops, free space on the card and on the frames' drive. Under the
    frames floor, frame saving stops (the run goes on); under the card floor, the run ends cleanly (robots stopped,
    logs closed, upload started) with a plain message;
  - run_plan, inside one drive, ends its run at a lower floor (CARD_HARD_GB): the last resort, normally never reached
    because the loop around it stops first.

    python3 tools/fleet/diskguard.py space                 # free space and the biggest folders (run-fleet-pi.sh space)
    python3 tools/fleet/diskguard.py preflight             # the start-up line; exit 1 when the card is under its floor
    python3 tools/fleet/diskguard.py where <session>       # the session's frames folder (doe-finish)

MIRLT_DRIVE=<folder> stands in for the USB drive (tests, dry runs); MIRLT_FAKE_FREE="card=0.8,drive=3" fakes the free
space (GB) the guard reads (tests, the dry run that shows the clean stop).
"""
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
GB = 1e9

# ---------------------------------------------------------------------------------------------- chosen numbers
CARD_STOP_DEFAULT_GB = 1.0   # (e12) the default floor; the site file's card_stop_gb (school 0.55) replaces it and
                             # MIRLT_CARD_STOP_GB still overrides both (card_stop_gb() below)
CARD_STOP_GB = float(os.environ.get("MIRLT_CARD_STOP_GB", "1.0"))   # (9 Nov, school: a 7 GB card shared with
                        # Jay-Sun's work had 0.63 GB free) MIRLT_CARD_STOP_GB lowers it for a short run; never under
                        # CARD_HARD_GB below. Was: 1.0, chosen: under this on the card, a long run (DOE loop, Monday mission, ladder) ends between
                        # loops. The 21:34 loop's logs (everything but frames) were 9.3 MB for 23 loops (0.4 MB a
                        # loop, ~80 MB for 4 h), so 1 GB is mostly room for the system (its journal, apt, Python, the
                        # start page): a full card also breaks those. No DOE: a safety margin; the logs' share can be
                        # derived from the next sessions' folders (logs only), the system's is not a robot question
CARD_HARD_GB = 0.5      # chosen: run_plan ends its run (every robot halted) under this, inside a drive: the last
                        # resort under CARD_STOP_GB. No DOE: a safety margin (the same derivation as above)
CARD_FRAMES_GB = 2.0    # chosen: frames on the card (no USB drive) stop under this, so 1 GB stays between the frames'
                        # stop and the loop's own. No DOE: follows from CARD_STOP_GB (logs grow 0.4 MB a loop, so the
                        # 1 GB gap is never used up by logs)
DRIVE_FRAMES_GB = 5.0   # chosen: frames on the USB drive stop when its free space, less the frames saved so far,
                        # is under this. "Less the frames saved so far": the upload zips a copy of them on the same
                        # drive (upload_session.py's .upload-tmp; 1.42 GB zip for 2026-10-27's frames). No DOE: a
                        # safety margin on a 256 GB drive (the recorder's full video, ~4 GB an hour a camera, is the
                        # other thing that fills it)
CHECK_S = 2.0           # chosen: the frames' own check runs with perf.csv (every 2 s; one statvfs call each, ~20 us).
                        # Frames grew 49 MB a minute on 2026-10-27 (FRAMES_GB_PER_H), so 2 s is under 2 MB. No DOE:
                        # far under any floor
FRAMES_GB_PER_H = 2.9   # measured, not chosen: 2026-10-27 21:34-22:03, both cameras' tracking-rate frames and clips
                        # zipped to 1.42 GB in 29 min. A 4 h loop: ~12 GB of frames, ~24 GB with the upload's copy


VERSION = "2026-11-10a"   # 2026-11-10a (e12, Nick 10 Nov): the card floor per site (card_stop_gb in the site file, school
                          # 0.55; MIRLT_CARD_STOP_GB still overrides, default 1.0): no `tmux set-environment` needed.
                          # No version string before


def card_stop_gb(site=None):
    """(e12) The card floor (GB) for the site in use: MIRLT_CARD_STOP_GB when set (the old override, kept), else the
    site file's card_stop_gb, else 1.0. Never under CARD_HARD_GB (run_plan's own last resort). (value, where from)."""
    env = os.environ.get("MIRLT_CARD_STOP_GB", "").strip()
    if env:
        try:
            return max(CARD_HARD_GB, float(env)), "MIRLT_CARD_STOP_GB"
        except ValueError:
            pass
    try:
        if site is None:
            if HERE not in sys.path:
                sys.path.insert(0, HERE)
            import sites as SI
            site = SI.load(None)
        v = (site or {}).get("card_stop_gb")
        if v is not None:
            return max(CARD_HARD_GB, float(v)), "the site file (%s)" % (site or {}).get("name", "?")
    except Exception:  # noqa: BLE001  (no site, a broken file: the default)
        pass
    return CARD_STOP_DEFAULT_GB, "the default"


CARD_STOP_GB, CARD_STOP_FROM = card_stop_gb()    # (e12) resolved once when imported (each run is its own process)


# (2026-11-03, risk review #6) the numbers above were measured with two cameras at 640 x 480. The frames saved and the
# recorder's video grow with the picture's pixels and the number of cameras: the frames' floors, the hour's estimate and
# the recorder's line scale with them (1280 x 960, two cameras: 4 x). At home (640 x 480, two cameras) nothing changes.
REF_PIXELS = 640 * 480
REF_CAMS = 2
REC_GB_PER_CAM_H_REF = 3.0   # the recorder's video at 640 x 480, 25 a second (camrec.DISK_GB_PER_CAM_10MIN: 0.5 per 10
                             # min, measured 2026-10-21/22)


def capture(env=None):
    """(width, height, fps, cameras) the recorder uses: the site's (the launcher's MIRLT_CAPTURE, MIRLT_CAPTURE_FPS,
    MIRLT_CAMS), else 640 x 480, 25 a second, two cameras."""
    env = os.environ if env is None else env
    w, h = 640, 480
    words = (env.get("MIRLT_CAPTURE") or "").split()
    for k, v in zip(words, words[1:]):
        if k == "--width" and v.isdigit():
            w = int(v)
        if k == "--height" and v.isdigit():
            h = int(v)
    try:
        fps = float(env.get("MIRLT_CAPTURE_FPS") or 25)
    except ValueError:
        fps = 25.0
    try:
        cams = int(env.get("MIRLT_CAMS") or REF_CAMS)
    except ValueError:
        cams = REF_CAMS
    return w, h, fps, cams


def capture_scale(env=None):
    """How much faster than the measured 640 x 480 two-camera numbers the frames grow (pixels x cameras); never under 1."""
    w, h, fps, cams = capture(env)
    return max(1.0, (w * h) / float(REF_PIXELS) * cams / float(REF_CAMS))


def rec_gb_per_h(env=None):
    """The recorder's video, GB an hour for all its cameras (camrec's rate scaled by pixels and pictures a second)."""
    w, h, fps, cams = capture(env)
    return REC_GB_PER_CAM_H_REF * (w * h) / float(REF_PIXELS) * fps / 25.0 * cams


def free_gb(path):
    """Free space (GB, for this user) of the filesystem holding path (its nearest existing parent); None on error."""
    p = os.path.abspath(path)
    while p and not os.path.exists(p):
        p = os.path.dirname(p)
    try:
        return shutil.disk_usage(p).free / GB
    except OSError:
        return None


def total_gb(path):
    try:
        return shutil.disk_usage(path).total / GB
    except OSError:
        return None


def drive_mount(user=None):
    """The USB drive the recorder uses (run-fleet-pi.sh start_recorder: the first writable folder under
    /media/<user>), or None. Only a real mount point counts here: a leftover folder under /media is on the card."""
    env = os.environ.get("MIRLT_DRIVE")
    if env:
        return env if os.path.isdir(env) else None
    user = user or os.environ.get("USER") or os.path.basename(os.path.expanduser("~"))
    base = os.path.join("/media", user)
    try:
        names = sorted(os.listdir(base))
    except OSError:
        return None
    for n in names:
        m = os.path.join(base, n)
        if os.path.isdir(m) and os.access(m, os.W_OK) and os.path.ismount(m):
            return m
    return None


def same_fs(a, b):
    def dev(p):
        p = os.path.abspath(p)
        while p and not os.path.exists(p):
            p = os.path.dirname(p)
        return os.stat(p).st_dev
    try:
        return dev(a) == dev(b)
    except OSError:
        return True


def frames_dir_for(session):
    """Where this session's frames go: (folder, on_drive). On the drive: <drive>/fleet-frames/<session name>/frames,
    and <session>/frames_where.txt says so (the folder keeps the name frames: the upload's asset stays
    rec-frames-small.zip). Not a direct child of the recordings folder: `stop` uploads the newest folder there."""
    d = drive_mount()
    if d:
        fd = os.path.join(d, "fleet-frames", os.path.basename(os.path.normpath(session)), "frames")
        try:
            os.makedirs(fd, exist_ok=True)
            os.makedirs(session, exist_ok=True)
            with open(os.path.join(session, "frames_where.txt"), "w") as f:
                f.write(fd + "\n")
            return fd, True
        except OSError:
            pass
    return os.path.join(session, "frames"), False


def where_frames(session):
    """The session's frames folder (frames_where.txt, else <session>/frames)."""
    p = os.path.join(session, "frames_where.txt")
    try:
        with open(p) as f:
            fd = f.read().strip()
        if fd:
            return fd
    except OSError:
        pass
    return os.path.join(session, "frames")


def folder_bytes(folder):
    n = 0
    for root, _, files in os.walk(folder):
        for fn in files:
            try:
                n += os.path.getsize(os.path.join(root, fn))
            except OSError:
                pass
    return n


def _fake():
    s = os.environ.get("MIRLT_FAKE_FREE", "")
    out = {}
    for kv in s.split(","):
        k, _, v = kv.partition("=")
        if k.strip() and v.strip():
            out[k.strip()] = float(v)
    return out


class Guard:
    """Free space on the card (the logs' filesystem) and on the frames' filesystem, against the floors.

    state(frames_bytes) -> dict(card_gb, frames_gb, frames_on_card, frames_ok, stop, why). `fake` (or MIRLT_FAKE_FREE)
    replaces the readings (GB) for tests and the dry run; fall(gb) lowers them (the dry run's falling disk)."""

    def __init__(self, card=None, frames_dir=None, card_stop_gb=CARD_STOP_GB, card_frames_gb=None,
                 drive_frames_gb=None, fake=None, scale=None):
        self.card = card or os.path.join(REPO, "logs")
        self.frames_dir = frames_dir
        # the frames' floors scale with the picture size and cameras (2026-11-03): the 1 GB between the frames' stop and
        # the card's stays; the frames' own share grows. Given floors are used as they are
        self.scale = capture_scale() if scale is None else float(scale)
        if card_frames_gb is None:
            card_frames_gb = card_stop_gb + (CARD_FRAMES_GB - CARD_STOP_GB) * self.scale
        if drive_frames_gb is None:
            drive_frames_gb = DRIVE_FRAMES_GB * self.scale
        self.card_stop_gb, self.card_frames_gb, self.drive_frames_gb = card_stop_gb, card_frames_gb, drive_frames_gb
        self.frames_on_card = frames_dir is None or same_fs(self.card, frames_dir)
        self.fake = dict(fake) if fake is not None else _fake()
        self.last = None

    def fall(self, gb):
        for k in list(self.fake):
            self.fake[k] = max(0.0, self.fake[k] - gb)

    def card_gb(self):
        return self.fake["card"] if "card" in self.fake else free_gb(self.card)

    def frames_gb(self):
        if self.frames_dir is None:
            return None
        if self.frames_on_card:
            return self.card_gb()
        return self.fake["drive"] if "drive" in self.fake else free_gb(self.frames_dir)

    def frames_floor(self):
        return self.card_frames_gb if self.frames_on_card else self.drive_frames_gb

    def state(self, frames_bytes=0):
        c, f = self.card_gb(), self.frames_gb()
        s = dict(card_gb=c, frames_gb=f, frames_on_card=self.frames_on_card, frames_ok=True, stop=False, why="")
        if f is not None:
            room = f - (0.0 if self.frames_on_card else frames_bytes / GB)
            if room < self.frames_floor():
                s["frames_ok"] = False
                s["why_frames"] = (
                    "the %s has %.1f GB free%s (frames stop under %.0f GB%s)" % (
                        "card" if self.frames_on_card else "USB drive", f,
                        "" if self.frames_on_card else ", %.1f GB of it needed for the upload's copy of the frames"
                        % (frames_bytes / GB),
                        self.frames_floor(), "" if self.frames_on_card else ", plus room for that copy"))
        if c is not None and c < self.card_stop_gb:
            s["stop"] = True
            s["why"] = "the Pi's card has only %.2f GB free (long runs end under %g GB, before it fills)" % (
                c, self.card_stop_gb)
        self.last = s
        return s

    def words(self, frames_on=True):
        """The start-up line: where frames go and how much room there is."""
        c = self.card_gb()
        parts = []
        if not frames_on or self.frames_dir is None:
            parts.append("Frames: not saved.")
        elif self.frames_on_card:
            parts.append("Frames go to %s ON THE PI'S CARD (%s): they stop under %.0f GB free." % (
                self.frames_dir, "no USB drive found under /media/%s" % os.environ.get("USER", "<user>")
                if drive_mount() is None else "the drive is on the same disk as the logs", self.card_frames_gb))
        else:
            parts.append("Frames go to %s (USB drive: %.1f GB free; they stop under %.0f GB plus room for the "
                         "upload's copy; a 4 h loop needs about %.0f GB there)." % (
                             self.frames_dir, self.frames_gb() or 0.0, self.drive_frames_gb,
                             8 * FRAMES_GB_PER_H * self.scale))
        parts.append("The Pi's card: %s GB free (the run ends cleanly under %g GB)." % (
            "?" if c is None else "%.1f" % c, self.card_stop_gb))
        return " ".join(parts)


def perf_cols(guard):
    """card_free_gb, frames_free_gb for perf.csv ('' when unknown)."""
    if guard is None:
        return "", ""
    c, f = guard.card_gb(), guard.frames_gb()
    return ("" if c is None else "%.2f" % c), ("" if f is None else "%.2f" % f)


# ---------------------------------------------------------------------------------------------- commands
def _human(n):
    return "%.1f GB" % (n / GB) if n >= 0.1 * GB else "%.0f MB" % (n / 1e6)


def biggest(folder, k=10):
    """The k biggest folders directly under folder: [(bytes, path)]."""
    out = []
    try:
        names = os.listdir(folder)
    except OSError:
        return out
    for n in names:
        p = os.path.join(folder, n)
        if os.path.isdir(p) and not os.path.islink(p):
            out.append((folder_bytes(p), p))
        elif os.path.isfile(p):
            out.append((os.path.getsize(p), p))
    return sorted(out, reverse=True)[:k]


def rec_out():
    try:
        with open("/tmp/fleet_rec_out") as f:
            p = f.read().strip()
        if p:
            return p
    except OSError:
        pass
    d = drive_mount()
    return os.path.join(d, "recordings") if d else os.path.expanduser("~/recordings")


def space(k=10, out=print):
    logs = os.path.join(REPO, "logs")
    c, ct = free_gb(logs), total_gb(logs)
    out("The Pi's card (logs/, the repo, the system): %.1f GB free of %.0f GB%s" % (
        c or 0, ct or 0, "   << UNDER THE %g GB FLOOR (%s): long runs will not start" % (CARD_STOP_GB, CARD_STOP_FROM)
        if c is not None and c < CARD_STOP_GB else ""))
    d = drive_mount()
    if d:
        out("The USB drive (%s): %.1f GB free of %.0f GB" % (d, free_gb(d) or 0, total_gb(d) or 0))
    else:
        out("The USB drive: not found under /media/%s (frames would go to the card)" % os.environ.get("USER", "<user>"))
    places = [("logs/ (on the card)", logs)]
    home_rec = os.path.expanduser("~/recordings")
    if os.path.isdir(home_rec):
        places.append(("~/recordings (on the card)", home_rec))
    home_tmp = os.path.expanduser("~/.upload-tmp")
    if os.path.isdir(home_tmp):
        places.append(("~/.upload-tmp (upload copies, on the card)", home_tmp))
    if d:
        for name, sub in (("recordings (USB drive)", "recordings"), ("fleet-frames (USB drive)", "fleet-frames"),
                          (".upload-tmp (upload copies, USB drive)", ".upload-tmp")):
            if os.path.isdir(os.path.join(d, sub)):
                places.append((name, os.path.join(d, sub)))
    for name, p in places:
        rows = biggest(p, k)
        out("")
        out("Biggest in %s (%s):" % (name, p))
        if not rows:
            out("  (empty)")
        for n, q in rows:
            out("  %9s  %s" % (_human(n), os.path.basename(q)))
    out("")
    out("To free the card: ./run-fleet-pi.sh clean-uploaded   (lists what is confirmed on GitHub; deletes nothing "
        "until you add --yes)")
    return 0


def rec_words(d=None, env=None):
    """The recorder's line: how fast its video grows and how long the place it writes to lasts."""
    w, h, fps, cams = capture(env)
    rate = rec_gb_per_h(env)
    place = os.path.join(d, "recordings") if d else os.path.expanduser("~/recordings")
    free = free_gb(place)
    hours = (free / rate) if (free is not None and rate > 0) else None
    line = "The recorder's video: about %.1f GB an hour (%d camera%s at %dx%d, %g a second) to %s%s" % (
        rate, cams, "" if cams == 1 else "s", w, h, fps, "the USB drive" if d else "THE PI'S CARD (no USB drive)",
        "" if hours is None else ": %.1f GB free there, about %.1f h" % (free, hours))
    low = hours is not None and hours < 1.0
    return line + ("." if not low else ". LOW: under an hour of video fits. Plug in a USB drive or free space "
                                       "(./run-fleet-pi.sh space)."), low


def preflight(session=None, out=print):
    """The start-up line for a run about to start; 1 when the card is under its floor (nothing should start)."""
    d = drive_mount()
    try:
        out(rec_words(d)[0])
    except Exception:     # noqa: BLE001  (a words line never stops a run)
        pass
    if d:
        frames = os.path.join(d, "fleet-frames", "<session>", "frames")
    else:
        frames = os.path.join(session or os.path.join(REPO, "logs", "<session>"), "frames")
    g = Guard(frames_dir=frames)        # (free_gb and same_fs read the nearest folder that exists)
    out(g.words())
    s = g.state()
    if s["stop"]:
        out("NOT STARTED: %s. See what is using it: ./run-fleet-pi.sh space" % s["why"])
        return 1
    if not s["frames_ok"]:
        out("(Frames will not be saved: %s.)" % s["why_frames"])
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    what = argv[0] if argv else "space"
    if what == "space":
        return space()
    if what == "preflight":
        return preflight(argv[1] if len(argv) > 1 else None)
    if what == "where":
        print(where_frames(argv[1]))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
