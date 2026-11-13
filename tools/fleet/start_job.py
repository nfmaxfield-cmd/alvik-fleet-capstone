"""Mission Control's Start screen (build e07, Nick 6 Nov): four big buttons (Calibrate, Free roam, Visit points,
Experiment routes), a red Stop that stops every robot, and plain status a newcomer understands. Nothing to type: the
place is found by itself (site_detect.py), the robots by their wiggle, the table and the area the camera reads by the
cameras. Everything typed or switched lives under Advanced (advanced()), out of the way.

Per run type (Nick's decisions 1-3):
  Calibrate          a roof-tag site: calib_run.py (every robot found, one after another; Calibrate is as it was);
                     home: the remote ladder's L0 + L1 (find by lights, calibrate)
  Free roam          home: freeroam.py (curves, firmware 11-05c+); a roof-tag site: every robot wanders over its share
                     of the readable points in a random order, curves (mission_run --roam-min)
  Visit points       straight lines, turns on the spot; every robot found by its wiggle, the readable points shared
  Experiment routes  Jay-Sun's routes (shakedown, e1-e4, in turn), straight lines, turns on the spot, any robot count
Practice: with no real camera (or Mission Control --dry) every button runs the simulator instead, with the guesses
broken on purpose (turned roof tags, a dark patch, the camera's clock off, a robot restarting), as the rehearsals do.
"""
VERSION = "2026-11-13e22"   # e22 (Nick 13 Nov): headline() (one plain status line: what runs, since when, each robot found / asleep / not heard, what it waits for), STOP also stops a free roam started from the terminal (its tmux session's Ctrl+C), needs-a-hand requests of an earlier run cleared, the start-failure words say the robots were woken first, note_sleep when robots are sent to sleep. Before: "2026-11-12mc-3"   # (Part 3) the Pi monitor beside practice and calibration runs. 1c: "2026-11-12mc-c"   # (1c) the Start screen's live table (live_view.py, /api/live). 1b: "2026-11-12mc-b"   # (Mission Control build, 12 Nov, 1b) uploads: upload_state (will upload / uploading / uploaded / failed / off / practice) on the Start screen; home free roam on the robots uploads when it ends (run-fleet-pi.sh freeroam-upload, the recorder left running). 1a: STOP: the StopWatch (stop_confirm.py): "stopping..." until every robot has confirmed, then "stopped" (never "ended"), its record in the run folder (stop.json); robot_refs (each nickname's WiFi name). Before: "2026-11-11e16"   # e16 (9a, home 11 Nov: "No camera model here: use a point below" on Right's needs-a-hand card): the click to locate a held robot uses whatever camera-to-board model the site has: the roof-tag fit where there is one, else the lights tracking's floor calibration (setup_conf: overhead_floorcal_*.json, side_floorcal_*.json) at the robot's top (e02_camera.ROBOT_TOP_CM). Before: "2026-11-10e14"   # e14: the clear-space question's words (clear_space) and the answer to every run (MIRLT_CLEAR_SPACE_OK / --clear-space); the Area line from the readability store (what is used, at what light, on how much evidence); Advanced "Battery floor"; practice runs keep their own readability store. e13: robots waiting for a hand (needs_hand) in the status. 2026-11-10a # 2026-11-10a (e12): Advanced "Curves in the calibration" (calib_curves) passed to calib_run
                           # (--arc-phase, --roam-motion); robots left at an edge in the status (edge_parked). No version
                           # string before this.
import glob
import json
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

KINDS = ("calibrate", "freeroam", "visit", "experiment")
TITLES = {"calibrate": "Calibrate", "freeroam": "Free roam", "visit": "Visit points", "experiment": "Experiment routes"}
ROAM_MIN = 10.0            # the one-button free roam at a tag site: about this many minutes. Chosen (a lesson's demo);
                           # Advanced changes it. No DOE: a time, not a measurement
HOME_ROAM_H = 4.0          # home free roam: freeroam.py's own cap (Nick: 4 h); every robot parks at 10 % battery
PRACTICE_ROAM_MIN = 3.0    # practice runs are short (the simulator runs in real time)
EXP_ORDER = ("e1", "e2", "e3", "e4")      # the one button cycles these; the shakedown (no camera, robots placed on
                                         # their homes by hand) is chosen in Advanced

ADV_DEFAULTS = {
    "sleep_when_done": True,       # Nick 6 Nov: robots sleep right away when a run ends
    "sleep_timer": False,          # the old 30-minute idle timer (kept, off)
    "sleep_timer_min": 30.0,
    "min_firmware": "",            # "" = the site's (calibration.min_firmware); "none" = no check (the old override)
    "corner_clicks": "auto",       # auto (the site's: clicks only when no reference tile is read) / on / off
    "surface_marks": "log",        # log (Nick 6 Nov) / act (e06's way) / off (before e06)
    "measure_trust": "auto",       # auto (cameras win when the reference tiles are read) / cameras / typed
    "allow_half_table": False,     # experiments on the half the camera cannot read (Dr. Sodhi's OK only)
    "roam_min": ROAM_MIN,
    "practice_robots": 2,
    "practice_breaks": True,       # practice runs break the guesses on purpose
    "practice_dark": "150,0,200,50",   # practice: a patch the camera cannot read (x0,y0,x1,y1 cm)
    "experiment": "next",          # next (shakedown, e1 ... e4 in turn) or one of EXP_ORDER
    "recover_reserve": "on",       # Nick 6 Nov: a lost robot's recovery path reserved in the others' plans
    "stop_lead": "on",             # Nick 6 Nov: missions use each robot's stop lead
    "show_all_screens": False,     # the old screens (Connect ... Production), kept
    "visit_speed_cms": 10.0,       # (e11, Jay-Sun's Experiment A tours) Visit points in an order: driving speed
    "visit_dwell_s": 8.0,          # ... and the stop at each point (his numbers, 9 Nov)
    "visit_chunk_cm": 0.0,         # (Nick 9 Nov) a tour's drives in pieces of at most this (0: each leg in one go, smooth)
    "visit_keep_unreadable": "on", # (Nick 9 Nov) a tour tries points the camera may not read (on) or skips them (off)
    "visit_order": "on",           # (e11, Nick 9 Nov) the Start screen's "Visit points in your order" picker (off: hidden)
    "practice_path": "real",       # (e11, 9 Nov) practice on the real path (robots answer with WiFi names, the real
                                   # launcher and camera models: mission_run --practice-real) or "dry" (before e11)
    "roam_motion": "curves",       # (Nick 9 Nov) free roam's driving: curves (the default) or stopturn (straight lines,
                                   # stop and turn on the spot), at home and at school
    "battery_floor": "",           # (e14) the battery floor for every run at this site, % ("" = the default: 25, free
                                   # roam 10); read by every run (engine_checks.battery_floor_used), tmux ones too
    "classic_page": False,         # (Mission Control build, Nick 12 Nov) the old page at / instead of the new map-first
                                   # page (kept, off; always at /classic). MIRLT_MC_PAGE=classic does the same
    "calib_curves": "auto",        # (e12, Nick 10 Nov) the calibration's curve phase: auto (robots on firmware 2026-11-05c+
                                   # when free roam drives curves), on, off (calib_run --arc-phase)
}


# (e11, Jay-Sun 9 Nov, Experiment A) the order presets on the visit card; more are saved from the page per site folder
# (visit_presets.json). "Jay-Sun custom" is his list as he gave it.
VISIT_PRESETS = {
    "In order (P01-P20)": ["P%02d" % k for k in range(1, 21)],
    "Reversed (P20-P01)": ["P%02d" % k for k in range(20, 0, -1)],
    "Jay-Sun custom": "P12 P18 P09 P14 P15 P13 P17 P11 P04 P19 P08 P01 P03 P20 P07 P05 P10 P16 P02 P06".split(),
}


# (e10, C5; Nick 7 Nov: "preferences take the form of a page of choices, with a short first-time quiz that fills it in")
# Saved per site on the Pi (<conf dir>/prefs/<site>.json). The essentials up front; everything else under Advanced,
# every switch from e08, e09 and e10 among them. Each switch's default is the build's default; "the switch" in a report
# is the other side. They reach the runs as the environment variables the modules read (PREF_ENV) and as freeroam.py's
# flags (prefs_args).
PREFS_DEFAULTS = {
    "quiz_done": False,
    # essentials
    "run_min": 0.0,                # 0: the site's usual (home: until the batteries are low, at most 4 h; school: 10 min)
    "caution": "normal",           # careful | normal | explore (explore: the probe-ahead setting on)
    "watching": "ask",             # yes (attended: a run may ask) | no (decides alone) | ask (asked at each start)
    "robots": "all",               # all, or names (home free roam)
    # advanced: e10
    "discovery": True,             # the start's proof drive (MIRLT_DISCOVERY)
    "workspace": "proven",         # proven | typed (the typed board's rectangle, as before e10)
    "probe_ahead": False,          # robots may slowly prove ground the camera cannot see
    "unc_split": True,             # the filter's doubt: tracked part x 2.45 + blind part x 8 (False: e09's 6 x sigma)
    "shuffle": True,               # the corner trap's last rung
    "picture_every_s": 60.0,       # at most one "unsure near an edge" picture per robot per this many seconds
    "school_roam": "freeroam",     # freeroam (e10: the home controller at school, one pipeline) | planner (e07, the switch)
    "camera_watch_method": "e10",  # e10 (averaged, noise-aware) | e09
    "queue_watch": True,           # (7 Nov run 2) every piece and its queued pieces re-checked against the fused position
    "mark_unc": True,              # (7 Nov run 2) a line trip near an edge is the edge unless the camera is sure enough
    "start_edge": True,            # (7 Nov run 2) a robot that starts over or at an edge is checked
    "start_edge_mode": "recover",  # recover (Nick 7 Nov: it first tries to back inward, asks only if that fails) | wait
    # advanced: e09
    "pose_source": "blend",        # blend | ekf (the filter in charge)
    "camera_watch": True,          # the camera watch (Nick 7 Nov, after e10: on; e09b had it off)
    "identity": "wiggle",          # wiggle (e09) | predict
    "gyro_heading": False,         # B3
    "old_tracking": False,         # the sources before e09
    "brick_watch": True,
    "lock": True,                  # the tag bridge's lock-on (school)
    # advanced: e08
    "edge_rule": "derived",        # derived | fixed
    "pose_rule": "widen",          # widen | wait2s
    "keepout_rule": "fade",        # fade | permanent
    "tilt_rule": "logs",           # logs | old (5 deg)
    "cam_edge_bias": True,         # the camera's 5 cm edge error in the clearance
    "silent_out": False,           # a silent robot out for the run (before e08)
    "tilt_recovery": True,
}
PREF_CHOICES = {
    "caution": ("careful", "normal", "explore"), "watching": ("yes", "no", "ask"), "workspace": ("proven", "typed"),
    "school_roam": ("planner", "freeroam"), "camera_watch_method": ("e10", "e09"), "pose_source": ("blend", "ekf"),
    "identity": ("wiggle", "predict"), "edge_rule": ("derived", "fixed"), "pose_rule": ("widen", "wait2s"),
    "keepout_rule": ("fade", "permanent"), "tilt_rule": ("logs", "old"), "start_edge_mode": ("recover", "wait")}
PREF_ENV = {           # preference -> (environment variable, value when True / the string itself)
    "discovery": ("MIRLT_DISCOVERY", "1", "0"), "unc_split": ("MIRLT_UNC_SPLIT", "1", "0"),
    "camera_watch": ("MIRLT_CAMERA_WATCH", "1", "0"), "gyro_heading": ("MIRLT_GYRO_HEADING", "1", "0"),
    "cam_edge_bias": ("MIRLT_CAM_EDGE_BIAS", "on", "off"), "pose_source": ("MIRLT_POSE_SOURCE",),
    "identity": ("MIRLT_IDENTITY",), "edge_rule": ("MIRLT_EDGE_RULE",), "pose_rule": ("MIRLT_POSE_RULE",),
    "keepout_rule": ("MIRLT_KEEPOUT_RULE",), "tilt_rule": ("MIRLT_TILT_RULE",),
    "camera_watch_method": ("MIRLT_CAMERA_WATCH_METHOD",), "school_roam": ("MIRLT_SCHOOL_ROAM",),
    "lock": ("MIRLT_TAG_LOCK", "1", "0"), "queue_watch": ("MIRLT_QUEUE_WATCH", "1", "0"),
    "mark_unc": ("MIRLT_MARK_UNC", "1", "0"), "start_edge": ("MIRLT_START_EDGE", "1", "0"),
    "start_edge_mode": ("MIRLT_START_EDGE_MODE",)}
QUIZ = [
    dict(id="where", q="Where are the robots today?",
         choices=[("auto", "Let it work it out (it looks at the table and the camera)"), ("home", "At home, on the 4 x 4 ft board"),
                  ("school", "At school, on the conference table")]),
    dict(id="length", q="How long should a run last?",
         choices=[("10", "A short demo (about 10 minutes)"), ("30", "A lesson (about 30 minutes)"),
                  ("0", "As long as the batteries last (home: up to 4 hours)")]),
    dict(id="watching", q="Will someone stay in the room with the robots?",
         choices=[("yes", "Yes, someone is always there"), ("ask", "Sometimes: ask me each time"),
                  ("no", "No, they run on their own")]),
    dict(id="caution", q="Near the table's edges, how careful should they be?",
         choices=[("careful", "Very careful: only where the camera clearly sees the table"),
                  ("normal", "Normal (recommended)"),
                  ("explore", "Let them slowly feel out parts the camera cannot see, with their own edge sensors")]),
    dict(id="robots", q="Which robots should drive?",
         choices=[("all", "Every robot that is switched on and on the table"), ("pick", "Only the ones I name")]),
]


def prefs_path(conf_dir, site):
    return os.path.join(os.path.expanduser(conf_dir or "~/.config/mirlt"), "prefs", "%s.json" % (site or "home"))


def prefs_env(p):
    """The environment variables a run reads, from the preferences."""
    out = {}
    for k, spec in PREF_ENV.items():
        v = p.get(k, PREFS_DEFAULTS.get(k))
        if len(spec) == 3:
            out[spec[0]] = spec[1] if v else spec[2]
        else:
            out[spec[0]] = str(v)
    return out


def prefs_args(p, with_robots=True):
    """freeroam.py's flags from the preferences (the ones with no environment variable)."""
    a = []
    if not p.get("discovery", True):
        a.append("--no-discovery")
    if p.get("workspace") == "typed":
        a.append("--typed-board")
    if p.get("probe_ahead") or p.get("caution") == "explore":
        a.append("--probe-ahead")
    if p.get("old_tracking"):
        a.append("--old-tracking")
    if not p.get("brick_watch", True):
        a.append("--no-brick-watch")
    if p.get("silent_out"):
        a.append("--silent-out")
    if not p.get("tilt_recovery", True):
        a.append("--no-tilt-recovery")
    if not p.get("shuffle", True):
        a.append("--no-shuffle")
    if float(p.get("picture_every_s", 60.0)) != 60.0:
        a += ["--picture-every-s", "%g" % float(p["picture_every_s"])]
    if p.get("caution") == "careful":
        a.append("--careful")
    r = str(p.get("robots") or "all").strip()
    if with_robots and r and r != "all":
        a += ["--robots", ",".join(x.strip() for x in r.replace(";", ",").split(",") if x.strip())]
    return a


def quiz_to_prefs(answers, prefs=None):
    """The first-time quiz's answers (question id -> choice id) into the preferences page (the rest left as it is)."""
    p = dict(PREFS_DEFAULTS if prefs is None else prefs)
    a = dict(answers or {})
    if a.get("length") is not None:
        p["run_min"] = float(a["length"])
    if a.get("watching") in PREF_CHOICES["watching"]:
        p["watching"] = a["watching"]
    if a.get("caution") in PREF_CHOICES["caution"]:
        p["caution"] = a["caution"]
        p["probe_ahead"] = a["caution"] == "explore"
    if a.get("robots") == "all":
        p["robots"] = "all"
    elif a.get("robots_names"):
        p["robots"] = str(a["robots_names"])
    p["quiz_done"] = True
    return p


def robot_refs(site, heard_file=None):
    """(12 Nov, nicknames) {nickname: reference}: each robot's WiFi name (heard_robots.json, matched by its address),
    else its address. The page shows the nickname; the reference stays visible beside it, smaller."""
    out = {}
    try:
        import stop_all as SA
        heard = SA.heard_from_file(heard_file, keep_h=24.0 * 365)
    except Exception:  # noqa: BLE001
        heard = {}
    by_ip = {ip: n for n, ip in heard.items()}
    for r in (site or {}).get("robots") or []:
        ip = r.get("ip")
        out[r["name"]] = by_ip.get(ip) or r.get("wifi_name") or ip or ""
    return out


def _alive(p):
    return p is not None and p.poll() is None


class ProcJob:
    """A launcher command in its own process group (home's ladder and free roam), its output in a file."""

    def __init__(self, kind, cmd, folder, env=None, practice=False):
        self.kind, self.cmd, self.folder, self.practice = kind, cmd, folder, practice
        os.makedirs(folder, exist_ok=True)
        self.out_path = os.path.join(folder, "stdout.txt")
        self.out = open(self.out_path, "w")
        self.p = subprocess.Popen(cmd, stdout=self.out, stderr=subprocess.STDOUT, cwd=REPO, start_new_session=True,
                                  env=dict(os.environ, **(env or {})))
        self.t0 = time.time()

    def running(self):
        return _alive(self.p)

    def stop(self):
        if self.running():
            try:
                os.killpg(self.p.pid, signal.SIGINT)     # its own Ctrl+C: every robot stops
            except OSError:
                pass

    def tail(self, n=12):
        try:
            with open(self.out_path, errors="replace") as f:
                lines = [ln.rstrip() for ln in f.readlines()[-200:] if ln.strip()]
        except OSError:
            return []
        return lines[-n:]


class Starter:
    def __init__(self, session, where=None, practice=None):
        self.S = session
        self.where = where or {"site": (session.site or {}).get("name", "home"), "kind": "home", "words": ""}
        self.practice_forced = practice
        self.job = None                 # a ProcJob (home), else the session's run or calibration
        self.active = None              # the kind running now (Start screen's)
        self.active_practice = False
        self.last = None                # {kind, practice, ended, words}
        self.lock = threading.Lock()
        self._end_lock = threading.Lock()
        self._robots_seen = None
        self._robots_t = 0.0
        self.adv_path = os.path.join(session.dir, "advanced.json")
        self.adv = dict(ADV_DEFAULTS)
        try:
            with open(self.adv_path) as f:
                self.adv.update({k: v for k, v in json.load(f).items() if k in ADV_DEFAULTS})
        except (OSError, ValueError):
            pass
        self.sleep_log = []
        self.deferred_sleep = None      # (e11) {at, why}: the sleep held back after a start where nothing moved
        self.load_prefs()

    # -- (e10, C5) preferences, saved per site on the Pi ------------------------------------------------------------
    def conf_dir(self):
        return (self.S.site or {}).get("conf_dir") or os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt")

    def site_name(self):
        return (self.where or {}).get("site") or (self.S.site or {}).get("name") or "home"

    def load_prefs(self):
        self.prefs = dict(PREFS_DEFAULTS)
        self.prefs_file = prefs_path(self.conf_dir(), self.site_name())
        try:
            with open(self.prefs_file) as f:
                self.prefs.update({k: v for k, v in json.load(f).items() if k in PREFS_DEFAULTS})
        except (OSError, ValueError):
            pass
        return self.prefs

    def save_prefs(self):
        os.makedirs(os.path.dirname(self.prefs_file), exist_ok=True)
        with open(self.prefs_file + ".tmp", "w") as f:
            json.dump(self.prefs, f, indent=1)
        os.replace(self.prefs_file + ".tmp", self.prefs_file)

    def set_prefs(self, d):
        for k, v in (d or {}).items():
            if k not in PREFS_DEFAULTS:
                raise ValueError("no such preference: %s" % k)
            t = type(PREFS_DEFAULTS[k])
            v = (str(v).lower() in ("1", "true", "on", "yes")) if t is bool else t(v)
            if k in PREF_CHOICES and v not in PREF_CHOICES[k]:
                raise ValueError("%s: one of %s" % (k, ", ".join(PREF_CHOICES[k])))
            self.prefs[k] = v
        if "caution" in (d or {}):
            self.prefs["probe_ahead"] = self.prefs["caution"] == "explore"
        self.save_prefs()
        return self.prefs

    def quiz(self, answers):
        self.prefs = quiz_to_prefs(answers, self.prefs)
        self.save_prefs()
        if (answers or {}).get("length") is not None and float(answers["length"]) > 0:
            self.adv["roam_min"] = float(answers["length"])        # the school's free roam minutes too
        return self.prefs

    def prefs_state(self):
        return {"values": dict(self.prefs), "defaults": dict(PREFS_DEFAULTS), "choices": {k: list(v) for k, v in
                                                                                   PREF_CHOICES.items()},
                "quiz": QUIZ, "file": self.prefs_file, "site": self.site_name()}

    # -- settings -------------------------------------------------------------------------------------------------
    def set_advanced(self, d):
        for k, v in (d or {}).items():
            if k not in ADV_DEFAULTS:
                raise ValueError("no such setting: %s" % k)
            t = type(ADV_DEFAULTS[k])
            self.adv[k] = (str(v).lower() in ("1", "true", "on", "yes")) if t is bool else t(v)
        with open(self.adv_path, "w") as f:
            json.dump(self.adv, f, indent=1)
        return self.adv

    def practice(self):
        if self.practice_forced is not None:
            return bool(self.practice_forced)
        return self.S.conn_default().get("mode") != "robots"

    def tags(self):
        return bool(self.S.tag)

    # -- what is seen ---------------------------------------------------------------------------------------------
    def robots_words(self):
        """(n, names, words) for the screen: during/after a run, the robots its wiggle found and named; before, the
        roof tags the camera sees (or the practice robots)."""
        names = self.bound_names()
        if names:
            return len(names), names, "Found %d robot%s: %s" % (len(names), "" if len(names) == 1 else "s",
                                                                ", ".join(names))
        if self.practice():
            n = int(self.adv.get("practice_robots") or 2)
            import wiggle_bind as WB
            nm = WB.auto_slots(self.S.site, n) if self.S.site else ["Left", "Center", "Right"][:n]
            return n, nm, "Practice: %d simulated robot%s (%s); each run finds them %s" % (
                n, "" if n == 1 else "s", ", ".join(nm), "by their wiggle" if self.tags() else "by their lights")
        n = self.tags_seen()
        if n is None:
            return 0, [], "Robots are found when a run starts (each one wiggles and is named)"
        if n == 0:
            return 0, [], "No robot seen on the table yet: put the robots on the table and switch them on"
        return n, [], "The camera sees %d roof tag%s: %d robot%s; each is named (Alpha, Bravo, ...) by its wiggle " \
                      "when a run starts" % (n, "" if n == 1 else "s", n, "" if n == 1 else "s")

    def tags_seen(self):
        """The roof tags the overhead camera sees now (a real tag site), refreshed every 10 s; None when unknown."""
        if not self.tags() or self.busy():
            return self._robots_seen
        if time.time() - self._robots_t < 10.0:
            return self._robots_seen
        self._robots_t = time.time()
        try:
            import cv2
            import numpy as np
            import mission_control as MC
            jpg = MC.grab_jpeg("127.0.0.1", 8000)
            if jpg is None:
                return self._robots_seen
            g = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            import tagcore as TC
            ids = list(int(i) for i in ((self.S.site.get("tags") or {}).get("ids") or {}).values()) + \
                [int(i) for i in ((self.S.site.get("tags") or {}).get("spares") or [])]
            det = TC.TagDetector((self.S.site.get("tags") or {}).get("family", TC.TAG_FAMILY), ids=ids, refine=False)
            self._robots_seen = len({d.id for d in det.detect_full(g)})
        except Exception:   # noqa: BLE001  (a status line never breaks the page)
            pass
        return self._robots_seen

    def bound_names(self):
        f = self.run_folder()
        if not f:
            return []
        for p in (os.path.join(f, "bind", "binding.json"), os.path.join(f, "binding.json")):
            try:
                with open(p) as fh:
                    rec = json.load(fh)
                return sorted(rec.get("slots") or {})
            except (OSError, ValueError):
                continue
        st = (self.S.calib.status() if self.active == "calibrate" and self.tags() else {}) or {}
        return [r["name"] for r in st.get("robots") or []] or ([st["robot"]] if st.get("robot") else [])

    def run_folder(self):
        if self.active in ("visit", "experiment", "freeroam") and self.S.run is not None and self.tags():
            return self.S.run.folder
        if self.active == "calibrate" and self.tags():
            return self.S.calib.folder
        return None

    def area_words(self):
        """What part of the table the runs use: the readable area the last run learned, else the camera model's. (e14)
        From the readability store: chosen for the light the last run measured, saying how much evidence it rests on
        and that other light's sessions are kept apart (10 Nov: a dim session had shrunk this to 26 cm)."""
        if not self.tags():
            return "Using the board the cameras see (the corners found at setup)"
        try:
            w = self.area_words_store()
            if w:
                return w
        except Exception:  # noqa: BLE001  (the e12/e13 words below)
            pass
        return self.area_words_e13()

    def area_words_store(self):
        import observe as OB
        import readmap_store as RS
        if not RS.enabled():
            return ""
        site = self.S.site or {}
        sd = site.get("conf_dir") or ""
        if self.practice():
            sd = os.path.join(self.S.dir, "practice-readmap")     # (e14) practice runs keep their own store
        d = RS.load(sd)
        if not d.get("sessions"):
            return ""
        tb = site.get("table") or {}
        L = float(tb.get("length_cm") or d.get("L") or 0)
        rm = OB.ReadMap(float(d.get("L") or L), float(d.get("W") or tb.get("width_cm") or 0))
        starts = sorted(glob.glob(os.path.join(self.S.dir, "runs", "*", "readmap_start.json")), key=os.path.getmtime)
        if starts:
            with open(starts[-1]) as f:
                rm.prior = OB.ReadMap.from_json(json.load(f)).prior     # the camera model's seed, as the runs use it
        used = sorted(glob.glob(os.path.join(self.S.dir, "runs", "*", "readmap_light_used.json")), key=os.path.getmtime)
        g = None
        if used:
            with open(used[-1]) as f:
                g = json.load(f).get("grey")
        if g is None:
            g = next((s_.get("grey") for s_ in reversed(d["sessions"]) if s_.get("grey") is not None), None)
        words = RS.choose(sd, rm, g, None)
        x = RS.readable_extent(rm, L)
        return ("Area used: where tags read at %s, about %.0f cm of the table's %.0f cm length (%s; a run in other "
                "light uses that light's evidence, measured again at its start)" % (
                    RS.light_words(g), x, L, words.split(": ", 1)[-1]))

    def area_words_e13(self):
        """(e12/e13) The readable area the last run learned (kept: MIRLT_READMAP_STORE=0)."""
        rm = None
        pats = sorted(glob.glob(os.path.join(self.S.dir, "runs", "*", "run", "readmap.json")), key=os.path.getmtime)
        if pats:
            try:
                with open(pats[-1]) as f:
                    rm = json.load(f)
            except (OSError, ValueError):
                rm = None
        tb = (self.S.site or {}).get("table") or {}
        L = float(tb.get("length_cm") or 0)
        if rm and rm.get("cells"):
            try:
                xs = [float(c[0]) for c in rm["cells"] if c[2] >= 0.6]
                if xs and L:
                    x1 = max(xs) + float(rm.get("cell_cm", 20)) / 2.0
                    return "Using the area the camera can see: about %.0f cm of the table's %.0f cm length" % (min(x1, L), L)
            except (TypeError, ValueError, IndexError, KeyError):
                pass
        return "Using the area the camera can see (found again at the start of every run)"

    def map_words(self):
        """The table as the cameras measured it, the typed value beside it when there is one, and disagreements."""
        if not self.tags():
            return ""
        p = os.path.join(self.S.tag.cal_dir, "site_fit.json")
        if self.practice():
            # a practice run measures with its own simulated cameras: the newest run's fit (rehearsal: the line still
            # said "measure at the first run" after one had)
            runs = sorted(glob.glob(os.path.join(self.S.dir, "runs", "*", "tagcal", "site_fit.json")), key=os.path.getmtime)
            p = runs[-1] if runs else ""
        try:
            with open(p) as f:
                fit = json.load(f)
        except (OSError, ValueError):
            return "The cameras measure the table by themselves at the first run (nothing to type)"
        L, W = fit.get("size_cm") or [0, 0]
        bad = [c for c in fit.get("checks") or [] if not c.get("ok") and c.get("what", "").startswith("table")]
        s = "Table measured by the cameras: %.0f x %.0f cm" % (L, W)
        if bad:
            s += " (your typed size differs; the cameras' is used and the difference is logged)"
        return s

    # -- buttons --------------------------------------------------------------------------------------------------
    def buttons(self):
        out = []
        tags = self.tags()
        has_points = bool(self.S.site and (self.S.site.get("points") or {}).get("use") == "typed")
        for k in KINDS:
            ok, why = True, ""
            if k in ("visit", "experiment") and not tags:
                ok, why = False, "Not on the home board: it has no points to visit (the points are on the school table)"
            elif k == "experiment" and not has_points:
                ok, why = False, "Needs Jay-Sun's points on the table (the school table)"
            out.append(dict(kind=k, title=TITLES[k], ok=ok, why=why, sub=self.sub_words(k)))
        return out

    def sub_words(self, k):
        if k == "calibrate":
            return "Each robot measures how its wheels and turns are off and fixes them. About 40 min a robot." \
                if self.tags() else "Finds the robots by their lights and calibrates each (the ladder's first steps)."
        if k == "freeroam":
            # (e11, 9 Nov) the button says the driving that will be used (Advanced > Free roam driving)
            how = "in straight lines (stop and turn on the spot)" if self.adv.get("roam_motion") == "stopturn" else \
                "on smooth curves"
            return ("All robots wander at once %s for about %.0f min." % (how, self.adv["roam_min"])) \
                if self.tags() else "All robots drive at once %s until their batteries reach 10 %%." % how
        if k == "visit":
            return "Every robot drives to the points in its share of the table, in straight lines."
        if k == "experiment":
            return "Jay-Sun's routes, straight lines: %s." % self.next_experiment_words()
        return ""

    def next_experiment(self):
        v = str(self.adv.get("experiment") or "next")
        if v in EXP_ORDER or v == "shakedown":
            return v
        last = None
        try:
            import experiments as EX
            d = EX.counter_read(self.S.site) if self.S.site else {}
            runs = d.get("runs") or []
            last = (runs[-1] or {}).get("experiment") if runs else None
        except Exception:   # noqa: BLE001
            last = None
        if last in EXP_ORDER:
            return EXP_ORDER[(EXP_ORDER.index(last) + 1) % len(EXP_ORDER)]
        return EXP_ORDER[0]

    def next_experiment_words(self):
        try:
            import experiments as EX
            return "next is %s" % EX.KINDS[self.next_experiment()]["name"]
        except Exception:   # noqa: BLE001
            return "next in turn"

    # -- runs -----------------------------------------------------------------------------------------------------
    def busy(self):
        return bool((self.job is not None and self.job.running()) or (self.S.run is not None and self.S.run.alive())
                    or self.S.calib.running())

    # -- (e11, Nick 9 Nov) Visit points in an order Nick chooses -------------------------------------------------------
    def visit_points(self, cam="overhead"):
        """The site's stickers for the order picker: their spots on the table (cm), where each sits in the camera's
        picture (px, through the site's camera model: the setup's fit on the robots, the practice room's in practice),
        and the ones the camera cannot read (flagged before the run; the run checks again)."""
        import argparse as _ap
        import math as _m
        import sticker_points as SP
        S = self.S
        site = S.site or {}
        pts, _z, _w = SP.points_cm(site)
        tb = site.get("table") or {}
        L, W = float(tb.get("length_cm") or 396.2), float(tb.get("width_cm") or 167.6)
        pr = self.practice()
        out = {"points": pts, "table": [L, W], "px": {}, "size": None, "unreadable": {}, "cam": cam,
               "practice": pr, "words": [], "presets": self.visit_presets()}
        model = None
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "experiment"))
            import e02_camera as CAM
            dirs = ([S.tag.sim_cal_dir()] if pr else []) + [S.tag.cal_dir]
            if pr:
                lr = (getattr(S, "last_runs", {}) or {}).values()
                dirs = [os.path.join(os.path.dirname(r_), "tagcal") for r_ in lr if r_] + dirs
            for d_ in dirs:
                p_ = os.path.join(d_, cam + ".json")
                if os.path.exists(p_):
                    model = CAM.load_camera(p_)
                    out["model"] = p_
                    break
        except Exception as e:      # noqa: BLE001
            out["words"].append("no camera model (%s): click the map instead" % e)
        if model is not None:
            z = float((site.get("tags") or {}).get("sticker_z_cm") or 0.0)
            for lab, (x, y) in pts.items():
                try:
                    u, v = model.project((float(x), float(y), z))
                    if _m.isfinite(u) and _m.isfinite(v):
                        out["px"][lab] = [round(float(u), 1), round(float(v), 1)]
                except Exception:   # noqa: BLE001
                    pass
            out["size"] = [int(v) for v in model.image_size]
        elif not out["words"]:
            out["words"].append("no camera model for %s yet (Setup, Measure): click the map instead" % cam)
        try:
            import mission_run as MR
            side = ((site.get("cameras") or {}).get("side") or {})
            a = _ap.Namespace(site_=site, dry=pr, practice_real=False, no_side=not side.get("use", True),
                              cam_size="%dx%d" % (int((site.get("capture") or {}).get("width") or 1280),
                                                  int((site.get("capture") or {}).get("height") or 960)),
                              seed=1, no_refs=False, tags_=None)
            rm = MR.run_readmap(a, None)
            if rm is not None:
                for lab, (x, y) in pts.items():
                    ok, why = rm.readable(float(x), float(y))
                    if not ok:
                        out["unreadable"][lab] = why
        except Exception as e:      # noqa: BLE001
            out["words"].append("(readable area not known: %s)" % e)
        return out

    def visit_presets(self):
        out = {k: list(v) for k, v in VISIT_PRESETS.items()}
        try:
            with open(os.path.join(self.S.dir, "visit_presets.json")) as f:
                out.update({str(k): [str(x) for x in v] for k, v in json.load(f).items()})
        except (OSError, ValueError, AttributeError):
            pass
        return out

    def save_visit_preset(self, name, order):
        name = str(name or "").strip()[:60]
        if not name:
            raise ValueError("Give the order a name")
        labs = [x for x in str(order or "").replace(" ", ",").split(",") if x.strip()]
        if not labs:
            raise ValueError("The order is empty")
        p = os.path.join(self.S.dir, "visit_presets.json")
        cur = {}
        try:
            with open(p) as f:
                cur = json.load(f)
        except (OSError, ValueError):
            pass
        cur[name] = labs
        with open(p, "w") as f:
            json.dump(cur, f, indent=1)
        return self.visit_presets()

    def start(self, kind, attended=False, order="", keep_unreadable=False, clear_space=None):
        if kind not in KINDS:
            raise ValueError("no such run: %s" % kind)
        with self.lock:
            if self.busy():
                raise ValueError("A run is going: press Stop first (or wait for it to finish).")
            b = next(x for x in self.buttons() if x["kind"] == kind)
            if not b["ok"]:
                raise ValueError(b["why"])
            pr = self.practice()
            if (self.S.site or {}).get("attended_only") and not pr and not attended:
                raise ValueError("At school the robots drive only while Jay-Sun is in the room: confirm he is here.")
            self.S.stop_asked = False
            self.active, self.active_practice = kind, pr
            self.started_t = time.time()
            if self.prefs.get("watching") in ("yes", "no"):
                attended = self.prefs["watching"] == "yes" or bool(attended and (self.S.site or {}).get("attended_only"))
            self.attended = bool(attended)          # (e08, A2) a person is there: runs may ask questions
            os.environ["MIRLT_ATTENDED"] = "1" if attended and not pr else "0"   # mission_run -> run_plan --attended
            os.environ.update(prefs_env(self.prefs))     # (e10, C5) the preferences reach every run this page starts
            # (e14) the person's answer to the clear-space question reaches the run's robot finding (none: wiggles)
            os.environ["MIRLT_CLEAR_SPACE_OK"] = "1" if clear_space else "0"
            self.clear_space_ = bool(clear_space)
            self.visit_order_ = {"order": str(order or "").strip(), "keep": bool(keep_unreadable)} \
                if kind == "visit" else None
            if self.tags():
                r = self.start_tags(kind, pr)
            else:
                r = self.start_home(kind, pr)
            self.last = None
            self.deferred_sleep = None      # (e11) a new run: the sleep held after a refused start is off
            return r

    def sim_breaks(self, n):
        """Practice runs break the guesses (Observe, then trust: the simulator's job): roof tags turned, a dark patch
        the camera cannot read, the camera's clock 0.7 s off, the first robot restarting 70 s in."""
        if not self.adv.get("practice_breaks", True):
            return []
        import wiggle_bind as WB
        names = WB.auto_slots(self.S.site, n)
        yaw = ",".join("%s=%d" % (nm, d) for nm, d in zip(names, (180, 90, -90, 45, 135)))
        cond = os.path.join(self.S.dir, "practice_cond.json")
        with open(cond, "w") as f:
            json.dump({"restart": {"name": names[0], "t": 70.0, "boot_s": 3.0}}, f)
        return ["--sim-tag-yaw", yaw, "--sim-dark", str(self.adv.get("practice_dark") or "150,0,200,50"), "--sim-clock-skew", "0.7", "--sim-cond", cond]

    def start_tags(self, kind, pr):
        S = self.S
        conn = "sim-cameras" if pr else "robots"
        if not pr and not os.path.exists(os.path.join(S.tag.cal_dir, "site_fit.json")):
            # (e07) nothing measured here yet: the cameras measure the table first (a few seconds, nothing to type)
            S.tag.measure("robots")
        n = int(self.adv.get("practice_robots") or 2)
        if kind == "calibrate":
            d = {"dry": pr}
            if pr:
                d["quick"] = True
                # as many practice robots as asked (each its own tag and spot), wheels that roll on (the stop lead)
                also = [("AL-2B7F", 3, "P01", 90), ("AL-E7DC", 4, "P06", 90), ("AL-EB54", 5, "P02", 90)]
                ex = []
                for nm, tg, pt, hd in also[:max(0, n - 1)]:
                    ex += ["--sim-also", "%s:%d:%s:%d" % (nm, tg, pt, hd)]
                if self.adv.get("practice_breaks", True):
                    ex += ["--sim-coast", "0.08"]
                d["extra"] = ex
            if self.adv.get("min_firmware") == "none":
                d["min_firmware"] = ""
            # (e12) the curve phase: Advanced "Curves in the calibration", decided by free roam's driving when auto
            d["extra"] = list(d.get("extra") or []) + ["--arc-phase", str(self.adv.get("calib_curves") or "auto"),
                                                       "--roam-motion", str(self.adv.get("roam_motion") or "curves")]
            if not pr:
                d["extra"] += ["--clear-space", "yes" if getattr(self, "clear_space_", False) else "no"]   # (e14)
            st = S.calib.start(d)
            return {"started": kind, "calib": st}
        extra = []
        if pr:
            extra += ["--sim-robots", str(n), "--sim-bind"] + self.sim_breaks(n)
            if self.adv.get("practice_path", "real") == "real":
                extra += ["--practice-real"]       # (e11) the school's own path, simulated robots
            extra += ["--readmap-store", os.path.join(self.S.dir, "practice-readmap")]   # (e14) practice's own store
        if self.adv.get("recover_reserve", "on") == "off":
            extra += ["--rp-arg=--recover-reserve=off"]
        if self.adv.get("surface_marks", "log") != "log":
            extra += ["--rp-arg=--surface-marks=%s" % self.adv["surface_marks"]]
        if self.adv.get("stop_lead", "on") == "off":
            extra += ["--no-stop-lead"]
        d = {"kind": kind, "conn": conn, "dry": pr, "sim_robots": n, "extra": extra}
        vo = getattr(self, "visit_order_", None)
        if kind == "visit" and vo and vo.get("order"):
            d["order"], d["keep_unreadable"] = vo["order"], vo["keep"]      # (e11) Nick's order
            d["extra"] = list(d["extra"]) + ["--visit-speed-cms", "%g" % float(self.adv.get("visit_speed_cms") or 10.0),
                                             "--visit-dwell-s", "%g" % float(self.adv.get("visit_dwell_s") or 8.0),
                                             "--visit-chunk-cm", "%g" % float(self.adv.get("visit_chunk_cm") or 0.0)]
        if kind == "freeroam":
            d["kind"] = "roam"
            d["motion"] = self.adv.get("roam_motion", "curves")   # (Nick 9 Nov)
            d["roam_min"] = PRACTICE_ROAM_MIN if pr else float(self.prefs.get("run_min") or self.adv.get("roam_min") or
                                                               ROAM_MIN)
        elif kind == "experiment":
            d["experiment"] = self.next_experiment()
            d["allow_half_table"] = bool(self.adv.get("allow_half_table"))
        st = S.start_cam_run(d["kind"], conn, d)
        return {"started": kind, "run": st}

    def start_home(self, kind, pr):
        """Home's runs through the launcher (its robots, cameras, calibration and dry mode are set up there)."""
        folder = os.path.join(self.S.dir, "runs", "%s-%s%s" % (time.strftime("%Y%m%d-%H%M%S"), kind, "-sim" if pr else ""))
        env = {"FLEET_DRY": "1" if pr else "0"}
        sh = os.path.join(REPO, "run-fleet-pi.sh")
        if kind == "calibrate":
            cmd = ["bash", "-c", "bash '%s' L0 && bash '%s' L1" % (sh, sh)]
        else:
            run_min = float(self.prefs.get("run_min") or 0.0)
            hrs = "%.2f" % ((PRACTICE_ROAM_MIN / 60.0) if pr else (run_min / 60.0 if run_min > 0 else HOME_ROAM_H))
            if pr:
                n = max(1, min(3, int(self.adv.get("practice_robots") or 2)))
                cmd = [sys.executable, os.path.join(HERE, "freeroam.py"), "run", "--dry", "--hours", hrs,
                       "--robots", ",".join(["Left", "Center", "Right"][:n]),
                       "--sim-minutes", "%.1f" % PRACTICE_ROAM_MIN, "--motion", self.adv.get("roam_motion", "curves"),
                       "--dir", folder] + \
                    (["--attended"] if getattr(self, "attended", False) else []) + \
                    prefs_args(self.prefs, with_robots=False)
                import calib_job as CJ
                if CJ._ns_ok():
                    cmd = ["unshare", "-rn"] + cmd
            else:
                cmd = ["bash", sh, "freeroam-run", hrs] + (["--attended"] if getattr(self, "attended", False) else []) + \
                    ["--motion", self.adv.get("roam_motion", "curves")] + prefs_args(self.prefs)
                if self.upload_on():
                    # (12 Nov, 1b) as the SSH way's freeroam-finish, but the recorder keeps running for Mission
                    # Control's camera: the run's logs and its own frames folder go up when it ends (a Stop included)
                    import shlex
                    cmd = ["bash", "-c", "%s; bash %s freeroam-upload" % (shlex.join(cmd), shlex.quote(sh))]
        env.update(prefs_env(self.prefs))
        self.job = ProcJob(kind, cmd, folder, env=env, practice=pr)
        if pr or kind == "calibrate":
            # (12 Nov) the Pi efficiency monitor; a real free roam gets its own from run-fleet-pi.sh freeroam-run (the
            # SSH way too), in free roam's own folder
            import mission_control as MC
            if hasattr(self.job, "p"):
                self.job.monitor = MC.monitor_for(folder, self.job.p.pid)
        return {"started": kind, "folder": folder}

    def upload_on(self):
        """(12 Nov, 1b) Upload when a run on the robots ends: the run settings' upload (on by default)."""
        return bool(((self.S.settings or {}).get("run") or {}).get("upload", True))

    def upload_state(self, folder=None, running=False, practice=None):
        """{"on", "state", "words", "link"}: will upload / uploading / uploaded / failed / off / practice, from the run's
        own output (upload_session.py's lines)."""
        pr = self.practice() if practice is None else practice
        on = self.upload_on()
        if pr:
            return {"on": on, "state": "practice", "words": "Practice: nothing is uploaded", "link": None}
        if not on:
            return {"on": False, "state": "off", "words": "Upload when it ends: off", "link": None}
        txt = ""
        for name in ("stdout.txt", "run_plan_stdout.txt", "mission_stdout.txt"):
            for d in (folder, os.path.dirname(folder or "")):
                try:
                    with open(os.path.join(d or "", name), errors="replace") as f:
                        txt += f.read()[-20000:]
                except OSError:
                    continue
        link = None
        if "All up: " in txt:
            link = txt.rsplit("All up: ", 1)[1].split()[0] if txt.rsplit("All up: ", 1)[1].split() else None
            return {"on": True, "state": "uploaded", "words": "Uploaded", "link": link}
        if "Upload failed" in txt or "upload failed" in txt:
            return {"on": True, "state": "failed", "words": "Upload failed: the files stay on the Pi (see the run's "
                                                            "log)", "link": None}
        if "Uploading " in txt or "uploading: " in txt:
            return {"on": True, "state": "uploading", "words": "Uploading the run's logs ...", "link": None}
        return {"on": True, "state": "will upload", "words": "Upload when it ends: on", "link": None}

    def stop_all(self):
        """The red Stop: every run this page started stops (its own Ctrl+C: `stop` to each of its robots), and on the
        robots a `stop` broadcast to every robot on the WiFi (stop_all.py), whatever holds them."""
        said = []
        import stop_confirm as SC
        jobs = self.stop_jobs()                # (12 Nov, F1) what was running, before it is stopped
        if self.job is not None and self.job.running():
            self.job.stop()
            said.append("the %s run" % TITLES.get(self.job.kind, self.job.kind))
        r = self.S.stop_run()
        if r:
            said.append("the run")
        pr = self.practice()
        try:                                   # (e22) a free roam started from the terminal: its own Ctrl+C too
            if subprocess.run(["tmux", "has-session", "-t", "freeroam"], capture_output=True, timeout=3).returncode == 0:
                subprocess.Popen(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "freeroam-stop"], cwd=REPO,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                said.append("the free roam started from the terminal (Ctrl+C to its tmux session)")
        except (OSError, subprocess.TimeoutExpired):
            pass
        if not pr:
            try:
                subprocess.Popen([sys.executable, os.path.join(HERE, "stop_all.py")], cwd=REPO,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                said.append("`stop` to every robot on the WiFi")
            except OSError:
                pass
        if jobs:
            # (Mission Control build, 12 Nov, decision 1a) "stopping..." until every robot has confirmed
            import stop_all as SA

            def send(ips):
                SA.stop_robots(list(ips) + SA.broadcast_addrs(), broadcast=True)
            self.stopwatch = SC.StopWatch(jobs, practice=pr, listen=None if pr else SA.listen_for_robots,
                                          send_stop=None if pr else send, say=lambda w: print("STOP: " + w, flush=True))
            self.stopwatch.kind = self.active
            self.stopwatch.start()
            threading.Thread(target=self._stop_record, args=(self.stopwatch,), daemon=True).start()
        return {"stopped": said or ["nothing was running"],
                "stop": self.stopwatch.state() if jobs else None}

    def stop_jobs(self):
        """(12 Nov) The process groups running now, each with its folder and the robots it drives (nickname, WiFi name
        or address), for the StopWatch."""
        import stop_confirm as SC
        robots = self.robots_for_stop()
        jobs = []
        if self.job is not None and self.job.running():
            jobs.append(SC.Job(self.job.kind, self.job.p.pid, self.job.folder, robots))
        run = self.S.run
        if run is not None and (run.alive() or SC.group_procs(run.p.pid)):
            jobs.append(SC.Job(run.kind, run.p.pid, getattr(run, "session", None) or run.folder,
                               [] if jobs else robots))
        cj = getattr(self.S, "calib", None)
        if cj is not None and cj.running():
            jobs.append(SC.Job("calibrate", cj.p.pid, cj.folder, [] if jobs else robots))
        return jobs

    def robots_for_stop(self):
        """[{name, ref, ip}]: the robots the run found (binding: nickname, WiFi name, address), else the site's."""
        out = []
        f = self.run_folder()
        for p in ((os.path.join(f, "bind", "binding.json"), os.path.join(f, "binding.json")) if f else ()):
            try:
                with open(p) as fh:
                    rec = json.load(fh)
                for r in rec.get("robots") or []:
                    out.append({"name": r.get("slot") or r.get("name_before"), "ref": r.get("wifi_name") or r.get("ip"),
                                "ip": r.get("ip")})
                if out:
                    return out
            except (OSError, ValueError):
                continue
        names = self.robots_words()[1]
        site = {r["name"]: r for r in (self.S.site or {}).get("robots") or []}
        refs = robot_refs(self.S.site)
        for n in names or list(site):
            out.append({"name": n, "ref": refs.get(n), "ip": (site.get(n) or {}).get("ip")})
        return out

    def _stop_record(self, sw):
        """The stop's record in the run's own folder (stop.json), once every robot has confirmed: the run's log says
        it was stopped (F1: "the run's log shows no stop")."""
        while sw.done_t is None:
            time.sleep(0.5)
        folder = next((j.folder for j in sw.jobs if j.folder and os.path.isdir(j.folder)), None)
        if folder:
            try:
                with open(os.path.join(folder, "stop.json"), "w") as f:
                    json.dump(sw.state(), f, indent=1)
            except OSError:
                pass

    # -- status ---------------------------------------------------------------------------------------------------
    def status(self):
        """Plain words for what is happening now, for someone who has never seen the page."""
        S = self.S
        kind = self.active
        running = self.busy()
        out = {"running": running, "kind": kind, "title": TITLES.get(kind, ""), "practice": self.active_practice,
               "lines": [], "elapsed_s": None, "points": None}
        try:                                   # (e08, A2) open questions from the run (safety_rules.Questions)
            import safety_rules as SR
            out["questions"] = SR.open_questions()[-3:] if running else []
        except Exception:  # noqa: BLE001
            out["questions"] = []
        if kind is None:
            self.deferred_sleep_due()
            return out
        out["elapsed_s"] = round(time.time() - getattr(self, "started_t", time.time()), 0)
        if self.job is not None and self.job.kind == kind and not self.tags():
            out["lines"] = self.job.tail(8)
            out["words"] = ("%s is running%s." % (TITLES[kind], " (practice)" if self.job.practice else "")) if \
                self.job.running() else "%s ended." % TITLES[kind]
            out["folder"] = self.job.folder
        elif kind == "calibrate":
            c = S.calib.status()
            import re
            out["words"] = "%s%s" % ({"find": "Finding the robots", "bind": "Naming each robot by its wiggle",
                                      "A": "Measuring each robot's wheels and turns", "B": "Working out the fix",
                                      "C": "Checking the fix on a tour", "park": "Parking", "done": "Done"}.get(
                c.get("phase"), "Starting"), ": " + c["words"] if c.get("words") else "")
            out["lines"] = [ln for ln in (c.get("tail") or "").splitlines() if ln.strip()][-8:]
            out["lines"] = [re.sub(r"\s+", " ", ln) for ln in out["lines"]]
            out["folder"] = c.get("folder")
        else:
            st = S.run.status() if S.run else {}
            msgs = st.get("messages") or []
            out["lines"] = [m[1] for m in msgs[-8:]]
            lv = st.get("live") or {}
            pts = lv.get("points")
            out["points"] = len(pts) if isinstance(pts, list) else pts      # (e07 rehearsal: a list showed as objects)
            out["words"] = ("%s is running%s." % (TITLES[kind], " (practice)" if self.active_practice else "")) \
                if st.get("running") else "%s ended." % TITLES[kind]
            out["folder"] = st.get("folder")
            if not out["lines"]:
                out["lines"] = self.mission_tail(st.get("folder"))
            refused = self.refusal_words(st.get("folder"))
            if refused and not st.get("running"):
                out["words"] = refused
                out["refused"] = True
        if not running and kind not in ("calibrate",) and not out.get("refused"):
            # (e11, 9 Nov school) a free roam that stopped at its start ("No robot answered") is a refused start too
            w_ = self.start_failure_words(out.get("folder"))
            if w_:
                out["words"], out["refused"] = w_, True
        out["upload"] = self.upload_state(out.get("folder"), running, self.active_practice)   # (12 Nov, 1b)
        sw = getattr(self, "stopwatch", None)
        if sw is not None and getattr(sw, "kind", None) == kind and sw.t0 >= getattr(self, "started_t", 0) - 1:
            # (12 Nov, F1) STOP was pressed on this run: "stopping..." until every robot confirmed, then "stopped"
            out["stop"] = sw.state()
            out["stopping"] = out["stop"]["phase"] == "stopping"
            out["words"] = "%s%s: %s" % (TITLES.get(kind, kind), " (practice)" if self.active_practice else "",
                                         out["stop"]["words"])
            out["stopped"] = not out["stopping"]
        end_now = False
        with self._end_lock:                  # the page's poll and the sleeper thread both ask: one sleep only
            if not running and self.last is None and not out.get("stopping"):   # (12 Nov) the robots' sleep waits
                                                                                # until every robot confirmed its stop
                self.last = {"kind": kind, "practice": self.active_practice, "ended": time.time(),
                             "words": out.get("words"), "refused": bool(out.get("refused"))}
                end_now = True
        if end_now:
            self.on_run_end(refused=bool(out.get("refused")) or self.nothing_moved(out.get("folder")))
        ep = self.edge_parked_words(out.get("folder"))
        if ep:
            out["edge_parked"] = ep          # (e12) robots left at an edge: someone must move them before the next run
        nh = self.edge_parked_words(out.get("folder"), key="needs_hand") if running else []
        if nh:
            out["needs_hand"] = nh           # (e13) robots waiting for a hand now: they rejoin once moved in
        self.deferred_sleep_due()
        ds = getattr(self, "deferred_sleep", None)
        if ds and not running:
            out["sleep_words"] = ("The robots were NOT put to sleep (nothing moved). They are put to sleep at %s if no "
                                  "run starts before then." % time.strftime("%H:%M", time.localtime(ds["at"])))
            out["words"] = "%s %s" % (out.get("words") or "", out["sleep_words"])
        return out

    # -- (e13, Nick 12:56) a held robot placed by a person's click -----------------------------------------------
    def place_model(self, cam):
        """The camera model Visit points uses for its webcam clicks (the setup's fit; the practice room's in practice)."""
        S = self.S
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "experiment"))
        import e02_camera as CAM
        pr = self.practice()
        dirs = ([S.tag.sim_cal_dir()] if (pr and S.tag) else []) + ([S.tag.cal_dir] if S.tag else [])
        if pr:
            lr = (getattr(S, "last_runs", {}) or {}).values()
            dirs = [os.path.join(os.path.dirname(r_), "tagcal") for r_ in lr if r_] + dirs
        kinds = self.__dict__.setdefault("place_kinds", {})
        for d_ in dirs:
            p_ = os.path.join(d_, cam + ".json")
            if os.path.exists(p_):
                kinds[cam] = "tags"
                return CAM.load_camera(p_)
        # (e16, 9a) no roof-tag model (home: lights tracking): the floor calibration the lights tracking uses, at the
        # height of the robot's top (where the click lands on the picture)
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
            import setup_conf as SCF
            p_ = SCF.overhead_calib(relative=False)[0] if cam == "overhead" else SCF.side_calib()[0]
            if p_ and os.path.exists(p_):
                kinds[cam] = "floorcal"
                return CAM.load_camera(p_)
        except Exception:  # noqa: BLE001  (no model: the map's points, as before)
            pass
        return None

    def place_z(self, cam):
        """(e16, 9a) The height a click is taken at: the roof tags' height on a tag model, the robot's top on a floor
        calibration (the lights tracking's plane)."""
        if (self.__dict__.get("place_kinds") or {}).get(cam) == "floorcal":
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "experiment"))
            import e02_camera as CAM
            return float(CAM.ROBOT_TOP_CM)
        return float(((self.S.site or {}).get("tags") or {}).get("height_cm") or 6.0)

    def place_state(self):
        import place_click as PC
        site = self.S.site or {}
        z = float((site.get("tags") or {}).get("height_cm") or 6.0)
        rq = PC.open_requests()
        # (e22, Nick 13 Nov 08:28) a request left by an earlier run (its robot no longer waits) is not shown as live:
        # only requests made since this run started, while it runs
        t0 = getattr(self, "started_t", None)
        stale = [q for q in rq if not self.busy() or t0 is None or float(q.get("t") or 0) < t0 - 1.0]
        rq = [q for q in rq if q not in stale]
        cams = {}
        for cam in ("overhead", "side"):
            m = None
            try:
                m = self.place_model(cam)
            except Exception:  # noqa: BLE001
                m = None
            if m is None:
                continue
            last = {}
            zc = self.place_z(cam)
            for q in rq:
                if q.get("last"):
                    u, v = m.project((q["last"][0], q["last"][1], zc))
                    if u == u and v == v:
                        last[q["robot"]] = [round(float(u), 1), round(float(v), 1)]
            cams[cam] = {"size": [int(v) for v in m.image_size], "last_px": last}
        pts = {}
        try:
            import sticker_points as SP
            pts = {k: list(v)[:2] for k, v in (SP.points_cm(site)[0] or {}).items()}
        except Exception:  # noqa: BLE001
            pts = {}
        return {"requests": rq, "cams": cams, "points": pts, "z_cm": z, "earlier_runs_cleared": len(stale)}

    def place_click(self, d, who=""):
        import place_click as PC
        robot, cam = str(d.get("robot")), str(d.get("cam") or "overhead")
        if d.get("point"):
            pts = self.place_state()["points"]
            if d["point"] not in pts:
                raise ValueError("no point %s on this table" % d["point"])
            return PC.point(robot, d["point"], pts[d["point"]], who=who)
        if d.get("accept"):
            return PC.accept_suggest(robot, who=who)          # the motion detector's spot, confirmed by the person
        m = self.place_model(cam)
        if m is None:
            raise ValueError("no camera model for %s: click the map's point instead" % cam)
        z = self.place_z(cam)
        try:                                                  # the picture clicked: shown beside the next one
            b = self.S.frame(cam) or (self.S.tag.picture_jpeg(cam) if self.S.tag else None)
            if b:
                with open(os.path.join(PC.qdir(), "place-%s-first-%s.jpg" % (robot, cam)), "wb") as f:
                    f.write(b)
        except Exception:  # noqa: BLE001
            pass
        return PC.click(robot, cam, m, float(d["u"]), float(d["v"]), z, who=who)

    def place_picture(self, robot, cam):
        import place_click as PC
        p = os.path.join(PC.qdir(), "place-%s-first-%s.jpg" % (robot, cam))
        if os.path.exists(p):
            with open(p, "rb") as f:
                return f.read()
        return None

    def edge_parked_words(self, folder, key="words"):
        """(e12, Nick 10 Nov) The newest edge_parked.json under the run's folder (free roam writes it when it parks a
        robot at an edge): one line per robot, naming it, where it is, and that someone must move it."""
        if not folder or not os.path.isdir(folder):
            return []
        ps = sorted(glob.glob(os.path.join(folder, "**", "edge_parked.json"), recursive=True), key=os.path.getmtime)
        if not ps:
            return []
        try:
            with open(ps[-1]) as f:
                return [str(w) for w in json.load(f).get(key) or []]
        except (OSError, ValueError):
            return []

    # (e11, Nick 9 Nov) a refused or failed start where nothing moved: no sleep at once (a sleeping robot needs a
    # power-cycle); put to sleep after DEFERRED_SLEEP_MIN minutes idle with no new run
    DEFERRED_SLEEP_MIN = 10.0      # Nick's number (9 Nov)

    def start_failure_words(self, folder):
        """The free-roam controller's own start failures, in plain words with what to fix (None: none found)."""
        lines = self.mission_tail(folder, n=60)
        txt = "\n".join(lines)
        if "No robot answered" in txt:
            return ("Not started, nothing moved: no robot answered the free-roam controller, even after waking them "
                    "through a whole check-in (a sleeping robot answers at its check-in). Check the robots are "
                    "switched on and on the WiFi (their names show on the Start screen), then press the button again.")
        if "No robot could join" in txt or "safety layers read back: nothing moved" in txt:
            return ("Not started, nothing moved: no robot could join the free roam (%s). Its log is in the folder "
                    "below." % next((x for x in reversed(lines) if "No robot" in x), "").strip())
        return None

    def nothing_moved(self, folder):
        """True when the run's log shows it never reached the point where the robots start (mission_run says 'the robots
        start now' just before it hands over)."""
        if not folder:
            return False
        p = os.path.join(folder, "mission_stdout.txt")
        try:
            with open(p, errors="replace") as f:
                return "the robots start now" not in f.read()
        except OSError:
            return False

    def deferred_sleep_due(self, now=None):
        ds = getattr(self, "deferred_sleep", None)
        if not ds:
            return False
        now = time.time() if now is None else now
        if self.busy():
            self.deferred_sleep = None
            return False
        if now >= ds["at"]:
            self.deferred_sleep = None
            self.sleep_robots("%.0f minutes idle after a start that did not move the robots (%s)" % (
                self.DEFERRED_SLEEP_MIN, ds["why"]))
            return True
        return False

    def refusal_words(self, folder):
        """(e07 rehearsal: a run refused at its start ended as just "ended", in command-line words) The reason a run
        did not start, in plain words, with what to press."""
        lines = self.mission_tail(folder, n=40)
        ln = next((x for x in reversed(lines) if "NOT STARTED" in x), None)
        if not ln:
            err = next((x for x in reversed(lines) if x.startswith(("planner.PlanError", "PlanError", "ValueError",
                                                                    "KeyError", "RuntimeError", "SystemExit"))), None)
            if err and "PlanError" in err:
                return ("Stopped before moving: no routes were found that keep the robots apart (%s). Try with fewer "
                        "robots on the table, or spread them out, then press the button again." % err.split(":", 1)[-1]
                        .strip())
            if err:
                return "Stopped with an error (nothing more moved): %s. Its log is in the folder below." % err
            return None
        if "camera cannot read" in ln and "experiment" in ln:
            return ("Not started, nothing moved: this route uses points the camera cannot read (with one camera it sees "
                    "only the TV-end half of the table). Jay-Sun's agreement: half the table needs Dr. Sodhi's OK. "
                    "With his OK, open Advanced and tick 'Experiments may use the half the camera cannot read', then "
                    "press Experiment routes again; or plug in camera 2.")
        txt = ln.split("NOT STARTED", 1)[1].lstrip(": ")
        return "Not started, nothing moved: %s" % txt

    def mission_tail(self, folder, n=8):
        for p in (os.path.join(folder or "", "mission_stdout.txt"), os.path.join(folder or "", "log.txt"),
                  os.path.join(folder or "", "run_plan_stdout.txt")):
            try:
                with open(p, errors="replace") as f:
                    return [ln.rstrip() for ln in f.readlines()[-n:] if ln.strip()]
            except OSError:
                continue
        return []

    # -- sleep (Nick 6 Nov: right away when a run ends; switch in Advanced; the 30-minute timer kept, off) ------------
    def headline(self):
        """(e22, Nick 13 Nov 08:28: "not very clear about its current status") One plain status line, always: what is
        running (or nothing), since when, each robot found / asleep / not heard, and what the run waits for."""
        import live_view as LV
        now = time.time()
        site = self.S.site or {}
        names = [r.get("name") for r in site.get("robots") or [] if r.get("name")]
        try:
            st = self.status()
        except Exception:  # noqa: BLE001
            st = {}
        folder, what, since = None, "", None
        if st.get("running"):
            folder = st.get("folder")
            since = getattr(self, "started_t", None)
            what = "%s%s is running (started here)" % (st.get("title") or st.get("kind") or "A run",
                                                        " (practice)" if st.get("practice") else "")
        else:
            term = LV.terminal_freeroam_folder()
            if term:
                folder = term
                try:
                    since = min(os.path.getmtime(os.path.join(term, f)) for f in os.listdir(term)) if \
                        os.listdir(term) else None
                except OSError:
                    since = None
                what = "A free roam started from the terminal is running (shown read-only; STOP stops it)"
            else:
                last = self.last or {}
                what = "Nothing is running" + (". Last: %s" % last.get("words") if last.get("words") else "")
        rob = {}
        waking = self.wake_words(folder) if folder else {}
        heard = {}
        try:
            import stop_all as SA
            p = SA.HEARD_FILE if hasattr(SA, "HEARD_FILE") else None
            with open(p or os.path.expanduser("~/.config/mirlt/heard_robots.json")) as f:
                heard = json.load(f) or {}
        except (OSError, ValueError, TypeError):
            heard = {}
        refs = robot_refs(site)
        import wake_robots as WR
        seen = {}
        if folder:
            for fn in ("follow.csv", "camera_track.csv"):          # its own pose / the camera, many times a second
                try:
                    if now - os.path.getmtime(os.path.join(folder, fn)) > 10.0:
                        continue
                except OSError:
                    continue
                for r_ in LV.tail_rows(os.path.join(folder, fn)):
                    if r_.get("robot"):
                        seen[r_["robot"]] = r_
            try:
                with open(os.path.join(folder, "wake.json")) as f:
                    wk = json.load(f)
                if now - float(wk.get("t") or 0) <= 10.0:
                    seen.update({k: v for k, v in (wk.get("robots") or {}).items() if v.get("awake")})
            except (OSError, ValueError, TypeError, AttributeError):
                pass
        for n in names:
            if n in seen and n not in waking:
                rob[n] = "found"
                continue
            if n in waking:
                rob[n] = waking[n]
                continue
            h = heard.get(n) or heard.get(refs.get(n, "")) or {}
            t = float(h.get("t") or 0) if isinstance(h, dict) else 0.0
            sl = WR.sleep_record(n)
            if folder and t and now - t < 60:
                rob[n] = "found"
            elif sl and float(sl.get("t") or 0) >= t - 5:
                nx = WR.next_checkin_s(n, now)
                rob[n] = "asleep" + (" (next check-in in ~%.0f s; a start wakes it)" % nx if nx is not None else "")
            elif t:
                rob[n] = "last heard %s" % time.strftime("%H:%M", time.localtime(t))
            else:
                rob[n] = "not heard yet"
        waiting = ""
        if waking:
            waiting = "waiting for sleeping robots to check in (they are woken then)"
        elif st.get("running") or folder:
            lines = list(st.get("lines") or [])
            w_ = [ln for ln in lines if ln.strip().lower().startswith("waiting")]
            waiting = w_[-1].strip() if w_ else ""
        words = what + (", since %s (%d min)" % (time.strftime("%H:%M", time.localtime(since)), (now - since) // 60)
                        if since else "") + "."
        if rob:
            words += " Robots: " + "; ".join("%s %s" % kv for kv in rob.items()) + "."
        if waiting:
            words += " Waiting: " + waiting.rstrip(".") + "."
        return {"words": words, "running": bool(st.get("running")), "terminal": bool(folder and not st.get("running")),
                "robots": rob, "waiting": waiting, "since": since}

    def wake_words(self, folder=None):
        """(e22) The run's wake.json (wake_robots.Waker): {robot: "asleep, waking (next check-in in ~N s)"} while a start
        waits for sleeping robots; {} otherwise."""
        folder = folder or self.run_folder()
        try:
            with open(os.path.join(folder, "wake.json")) as f:
                d = json.load(f)
            if time.time() - float(d.get("t") or 0) > 10.0:
                return {}
            return {n: v.get("words") for n, v in (d.get("robots") or {}).items() if not v.get("awake")}
        except (OSError, ValueError, TypeError, AttributeError):
            return {}

    def on_run_end(self, refused=False):
        if self.active_practice or not self.adv.get("sleep_when_done", True):
            return False
        if refused:
            # (e11, Nick 9 Nov) nothing moved: the robots stay awake for the next try; asleep after 10 min idle
            self.deferred_sleep = {"at": time.time() + self.DEFERRED_SLEEP_MIN * 60.0,
                                   "why": "refused or failed start"}
            print("The run did not start (nothing moved): the robots stay awake; put to sleep in %.0f min if no run "
                  "starts." % self.DEFERRED_SLEEP_MIN, flush=True)
            return False
        self.sleep_robots("the run ended")
        return True

    def sleep_robots(self, why):
        ck = int((self.S.site or {}).get("sleep_checkin_min", 10)) if self.S.site else 10
        try:
            subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "pi", "robot_sleep.py"), "sleep",
                            str(ck), "255.255.255.255"], timeout=20, capture_output=True)
            self.sleep_log.append(dict(t=time.time(), why=why))
            import wake_robots as WR
            WR.note_sleep(None, ck, why)          # (e22) the next start says when each checks in
            print("%s: every robot sent `sleep %d` (it wakes for 20 s every %d min; robot_sleep.py wake any brings "
                  "it back)" % (why, ck, ck), flush=True)
        except (OSError, subprocess.TimeoutExpired) as e:
            print("(sleep not sent: %s)" % e, flush=True)

    def state(self):
        n, names, rw = self.robots_words()
        pr = self.practice()
        return {"where": self.where, "practice": pr,
                "mode_words": "PRACTICE: the simulator (no real camera here)" if pr else "ROBOTS: the real robots",
                "robots": {"n": n, "names": names, "words": rw}, "area": self.area_words(), "map": self.map_words(),
                "buttons": self.buttons(), "status": self.status(), "advanced": dict(self.adv),
                "headline": self.headline(),
                "prefs": self.prefs_state(),
                "attended_only": bool((self.S.site or {}).get("attended_only")),
                "typed": self.typed_values(), "sites": self.site_choices(),
                "corners_needed": self.corners_needed(), "code": self.code_state(),
                "upload": self.upload_state(None, False, pr),           # (12 Nov, 1b) said before a run starts
                "upload_migrated": getattr(self.S, "upload_migrated", ""),
                "clear_space": self.clear_space_words()}

    def clear_space_words(self):
        """(e14) The question asked before a run: the clear space each robot's finding stroke needs (derived:
        camera_discovery.clearance, at the camera's own scale when it has a model)."""
        try:
            import camera_discovery as CD
            cpp = None
            try:
                from cameras import CameraModel
                p = os.path.join(self.S.tag.cal_dir, "overhead.json") if self.S.tag else ""
                if p and os.path.exists(p):
                    tb = (self.S.site or {}).get("table") or {}
                    m = CameraModel.load(p)
                    cpp = 1.0 / float(m.px_per_cm(float(tb.get("length_cm", 200)) / 2, float(tb.get("width_cm", 100))
                                                  / 2, 4.0))
            except Exception:  # noqa: BLE001
                cpp = None
            c = CD.clearance(cpp)
            return {"radius_cm": c["radius_cm"], "radius_in": c["radius_in"], "words": (
                "Each robot is found by a short move first (one at a time). Does every robot have nothing within %.0f cm "
                "(%.1f in) of its middle: more than a hand's width from the table's edge and from the next robot? "
                "OK: yes (each makes a %.0f cm stroke forward and back). Cancel: not sure (each only wiggles on the "
                "spot, which needs no extra room)." % (c["radius_cm"], c["radius_in"], c["stroke_cm"]))}
        except Exception as e:  # noqa: BLE001
            return {"words": "", "why": str(e)}

    def code_state(self):
        """(e12) Mission Control's own code changed since it started (mission_control.CodeWatch), or None."""
        cw = getattr(self.S, "code_watch", None)
        if cw is None:
            return None
        try:
            going = bool(self.busy() or (self.S.run is not None and self.S.run.alive()) or self.S.calib.running())
        except Exception:  # noqa: BLE001  (unsure: treat as going, never restart mid-run)
            going = True
        return cw.status(going)

    def corners_needed(self):
        """Corner clicking only when there is truly no other way: a tag site whose cameras read no reference tile
        and has no clicks (the fit cannot place the table), or home with no floor calibration. Words or None."""
        if self.practice():
            return None
        if self.tags():
            if os.path.exists(os.path.join(self.S.tag.cal_dir, "site_fit.json")):
                return None
            if (self.S.site.get("ref_tags") or {}).get("use", False):
                return None
            return "The cameras cannot find the table's corners by themselves here (no corner tiles): click them once."
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
            import setup_conf as SCF
            c = SCF.overhead_calib(relative=False)
            c = c[0] if isinstance(c, tuple) else c
            if c and os.path.exists(c):
                return None
        except Exception:   # noqa: BLE001
            return None
        return "The cameras do not know the board yet: click its 4 corners once."

    def typed_values(self):
        s = self.S.site or {}
        tb = s.get("table") or {}
        tg = s.get("tags") or {}
        pt = s.get("points") or {}
        return {"table_length_cm": tb.get("length_cm"), "table_width_cm": tb.get("width_cm"),
                "tag_height_cm": tg.get("height_cm"), "tag_cm": tg.get("tag_cm"),
                "points_file": pt.get("file"), "points_use": pt.get("use"),
                "min_firmware_site": (s.get("calibration") or {}).get("min_firmware"),
                "edge_in_cm": s.get("edge_in_cm")}

    def site_choices(self):
        try:
            import sites as SI
            import site_detect as SD
            return {"names": SI.names(), "pinned": SD.pinned_site()}
        except Exception:   # noqa: BLE001
            return {"names": [], "pinned": None}
