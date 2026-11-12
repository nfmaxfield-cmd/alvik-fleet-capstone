#!/usr/bin/env python3
"""Mission Control (docs/build-plans/mission-control.md, Phase 2, 2026-10-27): the six-screen page for the school demo
(Connect, Workspace, Warm-up, Mission, Run, Review) over the school engine in this folder:

    workspace.py   the table: rectangle, side kinds, spots A, B, C..., no-go zones
    warmup.py      the warm-up's tasks and what it found; mapview.py the engineer view's mapping picture
    missions.py    run modes and Monday's routine, as planner tasks
    planner.py     the plan, re-plans; run_plan.py drives it (robots or the simulator)

    python3 tools/fleet/mission_control.py --dir logs/school          # then open http://127.0.0.1:8095
    python3 tools/fleet/mission_control.py --dir logs/school --map tools/fleet/examples/school_map.json

Operator view (Details off): missions, spots, Run / Pause / Stop up front; every setting is still there behind
"More" on each screen (Nick, 2026-10-27 00:08: nothing locked or switched off). Details on: the settings open, plus
the engineer panels (the mapping picture, re-plans, timing).

Served on 127.0.0.1 only; changes are accepted only with this page's Host header. --dir keeps map.json,
mission.json, plan.json, settings.json and one folder per run (runs/<time>-<kind>/: run_plan's session).
"""
import argparse
import copy
import datetime as dt
import json
import math
import glob
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM     # noqa: E402
import planner as P       # noqa: E402
import missions as MS     # noqa: E402
import warmup as WU       # noqa: E402
import workspace as WS    # noqa: E402
import setup_job as SJ    # noqa: E402
import safety_group as SG  # noqa: E402  (e18: the Safety group, one box on the Run screen)

PAGE = os.path.join(HERE, "mission_control.html")              # the old page (kept: /classic)
TABLE_PAGE = os.path.join(HERE, "mission_control_table.html")  # (12 Nov) the map-first page, the default at /


def page_for(path, S=None, env=None):
    """Which page a path serves: /classic the old one, /table the new one, / the new one unless the old is chosen."""
    env = os.environ if env is None else env
    if path == "/classic":
        return PAGE
    if path == "/table":
        return TABLE_PAGE
    classic = (env.get("MIRLT_MC_PAGE") or "").strip().lower() == "classic"
    try:
        classic = classic or bool(S.start.adv.get("classic_page"))
    except AttributeError:
        pass
    return PAGE if classic or not os.path.exists(TABLE_PAGE) else TABLE_PAGE
PORT = 8095
PREVIEW_STEP_S = 0.25     # the preview's sampling of the plan (the slider's resolution). No DOE: display only
MONDAY_DEMO = os.environ.get("MIRLT_MONDAY_DEMO", "0") == "1"   # (e08, A10) the 2 Nov Monday demo's mission machinery
                        # (Monday blocks, the old 3-robot routine) is switched off by default: its modes are hidden on
                        # the Mission screen unless MIRLT_MONDAY_DEMO=1 (nothing deleted; the API still plans them)
TABLE_SIZES = [[61, 122], [70, 130], [76, 137]]   # Monday's table: at least 2 x 4 ft, at most 2.5 x 4.5 ft (Nick,
                                                  # 16:43); exact size Thursday. The Workspace screen's quick choices

# run settings (every one shown and editable on the page)
RUN_DEFAULTS = {
    "border_cm": 6.0,                 # run_plan.py BORDER_CM: the robots' own four-sided border (0 = off)
    "improve_budget_s": 6.0,          # run_plan.py: the background search for a shorter plan
    "timeout_s": 900.0,               # run_plan.py: stop everyone after this
    "edge_guard": "set safe_edge 3; set edge_dir 0; set edge_thr 150; set edge_n 3; set edge_bot_cm 11",
                                      # the edge guard as the runner sets it (e02.py SAFETY_SETTINGS)
    "sim_true_sides": "",             # simulator only: the table's real side kinds (fleet_sim --true-sides); "" = the map's
    "sim_true_objects": "",           # simulator only: boxes on the table the map does not know, "x,y,w,h; ..."
    "sim_brick": "",                  # simulator only: a brick drop, "auto@T" or "x,y[,angle]@T" (s after the start)
    "sim_brick_camera": True,         # simulator only: the brick is seen through the camera's what-changed watch
    "sim_washout": False,             # simulator through the cameras: the overhead's board washed out (27 Oct 11:20)
    "sim_drop_pct": 0.0,              # simulator through the cameras: lost packets, % each way
    "sim_dead": "",                   # simulator through the cameras: "Right@60": that robot dies 60 s in
    "sim_true_floor": "",             # simulator through the cameras, tags: the real table "x0,y0,x1,y1" when the map's
                                      # outline is wrong (feel out the table's tests); "" = the map's
    "farm": False,                    # robots: each camera tracked in its own process (trackfarm.py)
    "upload": True,                   # robots: upload the run's logs (a release with a tag of its own). (Mission Control
                                      # build, Nick 12 Nov, 1b) ON by default (F2: runs from the Start screen never
                                      # uploaded); a settings.json saved before with false is moved to true unless the
                                      # page recorded it as set by hand (settings "by_hand")
    "turn_pause": True,               # after each turn on the spot, stand until the cameras agree with the robot's own
                                      # pose, then drive (run_plan --turn-pause, 2026-10-29)
    "straighten": True,               # a robot off its start pose is turned on the spot or driven to its spot, then
                                      # checked again, instead of a stop (run_plan --straighten, 2026-10-29 15:41).
                                      # Not for the warm-up (it starts where the robots stand, as before)
    "tracking": "",                   # "" = the site's run_tracking; "tags" (roof AprilTags), "lights" (the robots' lights,
                                      # corners clicked) or "none" (no camera: odometry only, robots placed on their
                                      # homes by hand). The Run screen's selector (2026-11-03, Nick: "with or without
                                      # tags")
    "on_problem": "",                 # "" = the site's on_problem (e08: keep_going at home and school; stop is the switch); "keep_going" or
                                      # "stop" for the runs from this page (the Run screen's tick-box, 2026-11-03)
    "mirror": True,                   # a mirrored mission (Monday: Left and Right) is re-timed so they reach matching
                                      # points together: the late one speeds up, the leader slows or waits (run_plan's
                                      # mirror sync, 2026-10-29 20:06). Off: every leg at its planned speed
}
WARMUP_KNOBS = ("PROBE_SPACING_CM", "CORNER_PAUSE_S", "BARRIER_NEAR_CM", "SCALE_FLAG", "MAX_FRAMES")


def now_tag():
    return dt.datetime.now().strftime("%Y%m%d-%H%M%S")


START_IN_CM = 12.0     # a start spot carried to a new table size stays this far inside it (a robot's half-width and
                       # room to turn). No DOE: geometry


class Run:
    """run_plan.py in a subprocess (robots or the simulator), its session folder, Pause / Resume / Stop."""

    def __init__(self, kind, folder, cmd, env=None):
        self.kind, self.folder, self.cmd = kind, folder, cmd
        os.makedirs(folder, exist_ok=True)
        self.out = open(os.path.join(folder, "run_plan_stdout.txt"), "w")
        self.p = subprocess.Popen(cmd, stdout=self.out, stderr=subprocess.STDOUT, cwd=REPO,
                                  start_new_session=True, env=env)
        self.t0 = time.time()
        self.paused = False

    def alive(self):
        return self.p.poll() is None

    def signal(self, sig):
        if self.alive():
            try:
                os.kill(self.p.pid, sig)
            except OSError:
                pass

    def stop(self):
        """(Mission Control build, 12 Nov, F1) Ctrl+C to the run's whole process group, as a terminal's Ctrl+C: what
        drives the robots (run_plan, the simulator, the camera bridge) gets it at once, not only when mission_run
        passes it on. MIRLT_STOP_GROUP=0: the leader only, as before."""
        import stop_confirm as SC
        self.stop_t = time.time()
        if SC.group_on() and SC.signal_group(self.p.pid, signal.SIGINT):
            return
        self.signal(signal.SIGINT)

    def status(self):
        st = {"kind": self.kind, "folder": self.folder, "running": self.alive(), "paused": self.paused,
              "elapsed_s": round(time.time() - self.t0, 1), "exit": self.p.poll(), "mode": getattr(self, "mode", "sim")}
        if getattr(self, "session", None) and os.path.isdir(self.session):
            # mission_run.py: run_plan's session is in run/ (the camera bridge in bridge/, frames/, cpu.csv beside it)
            folder = self.session
        else:
            folder = self.folder
        lj = os.path.join(folder, "live.json")
        if os.path.exists(lj):
            try:
                with open(lj) as f:
                    st["live"] = json.load(f)
            except ValueError:
                pass
        tr = os.path.join(folder, "cam_track.csv")
        last = {}
        if os.path.exists(tr):
            with open(tr) as f:
                lines = f.readlines()[-60:]
            for ln in lines[1:] if lines and lines[0].startswith("t,") else lines:
                v = ln.strip().split(",")
                if len(v) >= 5 and v[1] not in ("robot",):
                    try:
                        last[v[1]] = [float(v[2]), float(v[3]), float(v[4]) if v[4] not in ("", "None") else 0.0]
                    except ValueError:
                        pass
        st["robots"] = last
        msgs, labels = [], {}
        cj = os.path.join(folder, "coordinator.jsonl")
        if os.path.exists(cj):
            with open(cj) as f:
                for ln in f.readlines()[-400:]:
                    try:
                        e = json.loads(ln)
                    except ValueError:
                        continue
                    if e.get("event") == "message":
                        msgs.append([round(e["t"] - self.t0, 1), e["text"]])
                    elif e.get("event") == "labelled_step" and e.get("state") == "started":
                        labels[e["robot"]] = e.get("label")
        st["messages"] = msgs[-40:]
        st["doing"] = labels
        sm = os.path.join(folder, "run_plan_summary.json")
        if os.path.exists(sm):
            try:
                with open(sm) as f:
                    st["summary"] = json.load(f)
            except ValueError:
                pass
        return st


# -- which mode the Connect screen opens on (Nick, 2026-10-29): "Robots" on the Pi when the real cameras answer, the
# simulator for dry runs (FLEET_DRY=1, --dry) and off the Pi. The page says which, and why, in its header.
REAL_CAMERA_PORTS = (8000, 8001)    # the recorder's overhead and side /live streams (camrec.py) on the Pi


def _port_answers(port, host="127.0.0.1", timeout=0.3):
    import socket as _s
    try:
        _s.create_connection((host, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def conn_default(dry=None, probe=_port_answers):
    """{"mode": "robots" | "sim", "why": plain words}. dry: forced dry (--dry or FLEET_DRY=1)."""
    if dry is None:
        dry = os.environ.get("FLEET_DRY", "0") == "1"
    if dry:
        return {"mode": "sim", "why": "dry run (FLEET_DRY=1): the simulator"}
    up = [p for p in REAL_CAMERA_PORTS if probe(p)]
    if up:
        cams = " and ".join("overhead" if p == 8000 else "side" for p in up)
        return {"mode": "robots", "why": "real cameras answering on this Pi (%s): the robots" % cams}
    return {"mode": "sim", "why": "no real camera answering here (not the Pi, or its recorder is not running): the "
                                  "simulator"}


def monitor_for(folder, pgid):
    """(12 Nov, Part 3) The Pi efficiency monitor beside a run this page starts (tools/pi/pi_monitor.py): its own
    process, low priority, ending with the run; pi_monitor.csv in the run's folder. Never stops a run."""
    if not pgid:
        return None                          # nothing to end with: never a monitor that runs on by itself
    try:
        pi = os.path.join(os.path.dirname(HERE), "pi")
        if pi not in sys.path:
            sys.path.insert(0, pi)
        import pi_monitor as PM
        return PM.start_for(folder, pgid=pgid)
    except Exception as e:  # noqa: BLE001
        print("(the Pi monitor did not start: %s)" % e, flush=True)
        return None


def pi_health_state(S):
    """(12 Nov, Part 3) The "Pi health" panel: the run's own pi_monitor.csv while a run goes (or the last run's), else
    a reading made here and now. {"source", "folder", "health": pi_monitor.health(...), "series": the last rows}."""
    pi = os.path.join(os.path.dirname(HERE), "pi")
    if pi not in sys.path:
        sys.path.insert(0, pi)
    import pi_monitor as PM
    folders = []
    st = getattr(S, "start", None)
    if st is not None and st.job is not None:
        import live_view as LV
        folders += [st.job.folder, LV.freeroam_folder(st) or ""]
    if S.run is not None:
        folders += [S.run.folder]
    cj = getattr(S, "calib", None)
    if cj is not None and getattr(cj, "folder", None):
        folders += [cj.folder]
    busy = run_going(S)
    best = None
    for f in folders:
        p = os.path.join(f or "", PM.NAME)
        if f and os.path.exists(p) and (best is None or os.path.getmtime(p) > os.path.getmtime(best)):
            best = p
    if best and (busy or time.time() - os.path.getmtime(best) < 120):
        rows = PM.rows_of(best)
        src = "this run's pi_monitor.csv (every 2 s)"
    else:
        if getattr(S, "_pi_idle", None) is None:
            S._pi_idle = PM.Idle(S.dir)
            time.sleep(0.3)
        rows = S._pi_idle.now()
        best, src = None, "read now by Mission Control (no run going)"
    keys = ("t", "core0", "core1", "core2", "core3", "temp_c", "lag_overhead_s", "lag_side_s", "loop_work_ms_p95",
            "det_share_pct", "det_cap_pct", "throttle_now")
    return {"source": src, "file": best, "running": busy, "health": PM.health(rows),
            "series": [{k: r.get(k) for k in keys} for r in rows[-90:]], "version": PM.VERSION}


def upload_default_migrate(settings):
    """(Mission Control build, 12 Nov, 1b) Uploads are on by default. settings.json is saved whole, so a file saved
    before this build holds "upload": false whether or not anyone chose it. Unless the page recorded upload as set by
    hand, it follows the new default. Returns words when it moved it, else ""."""
    run = settings.setdefault("run", {})
    if run.get("upload") is False and "run.upload" not in (settings.get("by_hand") or {}):
        run["upload"] = RUN_DEFAULTS["upload"]
        w = ("Uploads are now on by default (12 Nov): this site's saved 'upload: false' was never set by hand, so runs "
             "on the robots upload their logs when they end. Turn it off in Settings > This run.")
        print(w, flush=True)
        return w
    return ""


def mark_by_hand(settings, d):
    """The run settings a person changed on the page (only those whose value really changed)."""
    bh = settings.setdefault("by_hand", {})
    for k, v in ((d or {}).get("run") or {}).items():
        if k == "upload" and settings.get("run", {}).get(k) != v:
            bh["run.upload"] = round(time.time(), 1)


class Session:
    def __init__(self, folder, map_path=None, port=PORT):
        self.dir = os.path.abspath(folder)
        os.makedirs(os.path.join(self.dir, "runs"), exist_ok=True)
        self.lock = threading.Lock()
        self.port = port
        self.map_path = os.path.join(self.dir, "map.json")
        if map_path:
            self.map = FM.load(map_path)
            self.map["tasks"] = {}            # the workspace has no tasks: missions bring their own
            self.save_map()
        elif os.path.exists(self.map_path):
            self.map = FM.load(self.map_path)
        else:
            self.map = WS.blank_map((0, 0, 152, 76), [("Left", [30, 18, 90]), ("Center", [76, 18, 90]),
                                                      ("Right", [122, 18, 90])])
            self.save_map()
        self.settings = {"planner": {}, "mission": {}, "run": dict(RUN_DEFAULTS),
                         "warmup": {k: getattr(WU, k) for k in WARMUP_KNOBS},
                         "safety": SG.settings_defaults(),       # (e18) every safety switch, "auto" by default
                         "by_hand": {}}         # (12 Nov) run settings a person changed on the page: {"run.upload": t}
        sp = os.path.join(self.dir, "settings.json")
        if os.path.exists(sp):
            try:
                with open(sp) as f:
                    saved = json.load(f)
                for k in self.settings:
                    self.settings[k].update(saved.get(k) or {})
            except ValueError:
                pass
        self.upload_migrated = upload_default_migrate(self.settings)
        self.gap_notices = []
        self.lock_gaps("the saved settings.json")
        self.mission = None          # the mission map (tasks) last made
        self.program = None          # the block program on the Mission screen (default: Monday's, missions.py)
        pp = os.path.join(self.dir, "program.json")
        if os.path.exists(pp):
            try:
                with open(pp) as f:
                    self.program = json.load(f)
            except ValueError:
                self.program = None
        self.plan = None
        self.preview = None
        self.warm_map = None
        self.warm_result = None
        self.run = None
        import calib_job as CJ
        self.calib = CJ.CalibJob(self)   # the calibration run's card on the Production screen (2026-11-04)
        self.last_runs = {}          # kind -> folder
        self.corner = SJ.CornerSetup(self)   # the corners-only setup (2026-10-29): Setup screen, tools/fleet/setup_job.py
        # the site (2026-11-02): home's lights, or a site with AprilTags on the roofs (tag_job.py: Cameras & table,
        # Production, the sync card)
        self.site, self.tag = None, None
        try:
            import sites as SI
            self.site = SI.load(getattr(self, "site_name", None))
        except (SystemExit, ValueError, OSError) as e:
            if getattr(e, "site_file_error", False):
                raise                    # a broken site file stops loudly (2026-11-03), never home's ways in silence
            print("site not read (%s): home's ways" % e, flush=True)
        self.feel = None             # Feel out the table (2026-11-02): feel_job.py, a tag site's first run on a table
        self.last_production = None  # the Production screen's last orders (planned again on a felt outline)
        if self.site and self.site.get("tracking") == "tags":
            import tag_job as TJ
            import feel_job as FJ
            self.tag = TJ.TagJob(self, self.site)
            if self.site.get("feel_out", True):
                self.feel = FJ.FeelJob(self)      # a site may switch it off: "feel_out": false in its profile
            if not map_path and not os.path.exists(os.path.join(self.dir, "map.json.tagsite")):
                # a new tag site's folder: the site's table with its homes (Measure remakes it from the cameras)
                import production as PR
                tb = self.site.get("table") or {}
                m, _ = PR.site_map((float(tb.get("length_cm") or 396.24), float(tb.get("width_cm") or 167.64)),
                                   [r["name"] for r in self.site["robots"]])
                if not [r["name"] for r in self.map.get("robots", [])] == [r["name"] for r in self.site["robots"]]:
                    self.map = m
                    self.save_map()
                open(os.path.join(self.dir, "map.json.tagsite"), "w").close()
            self.load_typed_points()

    def load_typed_points(self, force=False):
        """A site with typed production points (Jay's sticker sheet, 2026-11-03): the map with its points, homes and
        raised box, and the default production mission loaded ready to run. Made again whenever the points file or the
        site's settings change (a marker with their fingerprint), never over a Measure of the same inputs."""
        import sticker_points as SP
        if not (self.site and SP.typed_on(self.site)):
            return None
        import hashlib
        import production as PR
        with open(SP.points_file(self.site), "rb") as f:
            fp = hashlib.sha1(f.read() + json.dumps({k: self.site.get(k) for k in (
                "points", "edge_in_cm", "homes", "keepouts", "production", "table", "experiments")}, sort_keys=True,
                default=str).encode()).hexdigest()[:12]
        mark = os.path.join(self.dir, "map.json.typed-" + fp)
        if os.path.exists(mark) and not force and self.program is not None:
            pr = self.program.get("production") or {}
            if pr.get("orders") and self.last_production is None:
                # a restart: the orders in use stay the Production screen's (never the first-3 default)
                self.last_production = {"orders": pr["orders"], "dwell_s": pr.get("dwell_s", 3.0),
                                        "home": pr.get("home", True)}
            return None
        m, words, prog, orders = PR.site_production(self.site)
        self.map = m
        self.save_map()
        self.program = prog
        with open(os.path.join(self.dir, "program.json"), "w") as f:
            json.dump(prog, f, indent=1)
        self.last_production = {"orders": orders, "dwell_s": prog["production"]["dwell_s"], "home": True}
        self.typed_words = words
        for old in [x for x in os.listdir(self.dir) if x.startswith("map.json.typed-")]:
            os.remove(os.path.join(self.dir, old))
        open(mark, "w").close()
        try:
            self.make_mission({"mode": "program", "program": prog})
        except Exception as e:     # noqa: BLE001  (the page shows Plan's error when Nick presses it)
            print("typed points: the default mission was not planned (%s)" % e, flush=True)
        return words

    # -- files -------------------------------------------------------------
    def save_map(self):
        FM.save(self.map, self.map_path)

    def safety_env(self):
        """(e18) The run's environment: the Safety box's switches that are not "auto" (the rest decided automatically)."""
        ch = SG.env_from_settings(self.settings.get("safety") or {})
        return dict(os.environ, **ch) if ch else None

    def save_settings(self):
        with open(os.path.join(self.dir, "settings.json"), "w") as f:
            json.dump(self.settings, f, indent=1)
        if not getattr(self, "_safety_file_off", False):
            SG.write_file(self.settings.get("safety") or {})     # (e18) every run on this Pi uses the Safety box

    def lock_gaps(self, where):
        """The 8 cm lock (Nick, 2026-10-29): no robot gap under the safety net (or the mission's chicken_min_cm) on
        the planner or mission screens, or in settings.json. A lower value is raised, saved, and said on the page
        (gap_notices) and in the terminal. The planner screen's separation_cm shows the net, not the planner's own 5."""
        out = MS.gap_lock_settings(self.settings["mission"], where="%s, mission settings" % where)
        floor = MS.gap_floor(MS.settings(self.settings["mission"]))
        out += MS.gap_lock_planner(self.settings["planner"], floor, where="%s, planner settings" % where) \
            if "separation_cm" in self.settings["planner"] else []
        self.gap_notices = out
        for w in out:
            print("8 cm LOCK: " + w, flush=True)
        if out:
            self.save_settings()
        return out

    def apply_warmup_knobs(self):
        for k, v in self.settings["warmup"].items():
            if k in WARMUP_KNOBS:
                setattr(WU, k, type(getattr(WU, k))(v))

    # -- state -------------------------------------------------------------
    def state(self):
        m = self.map
        out = {"map": m, "words": WS.words(m), "settings": self.settings,
               "defaults": {"planner": dict(P.DEFAULT_SETTINGS, separation_cm=MS.gap_floor()), "mission": dict(MS.SETTINGS),
                            "run": dict(RUN_DEFAULTS),
                            "warmup": {k: getattr(WU, k) for k in WARMUP_KNOBS},
                            "safety": SG.settings_defaults()},
               "safety_info": [dict(env=e, default=d, old=o, what=w) for e, d, o, _a, w, _b in SG.SWITCHES],
               "least_open_room_cm": round(MS.least_open_room(m), 2) if m.get("sides") else None,
               "program": self.program or MS.monday_program(), "blocks": list(MS.BLOCKS),
               "table_sizes": TABLE_SIZES, "setup": self.setup_state(), "surface": self.surface(),
               "mission": (self.mission or {}).get("mission"), "preview": self.preview,
               "warmup": self.warm_result, "run": self.run.status() if self.run else None,
               "last_runs": self.last_runs, "gap_notices": list(getattr(self, "gap_notices", [])),
               "conn_default": self.conn_default(), "corner_setup": self.corner.state(),
               "setup_way": self.corner.way}
        if self.site:
            st = self.site
            out["site"] = {"name": st["name"], "title": st["title"], "tracking": st["tracking"],
                           "attended_only": bool(st.get("attended_only")),
                           "monday_demo": MONDAY_DEMO,
                           "robots": [r["name"] for r in st["robots"]],
                           "colours": {r["name"]: r["colour"] for r in st["robots"]},
                           "cameras": {k: v.get("label", k) for k, v in st["cameras"].items()},
                           "capture": st["capture"], "tags": st.get("tags"), "table": st.get("table"),
                           "on_problem": st.get("on_problem", "keep_going"), "feel_out_trust": st.get("feel_out_trust"),
                           "lost_robot_points": st.get("lost_robot_points"),
                           "run_tracking": st.get("run_tracking", st["tracking"]),
                           "points_typed": bool((st.get("points") or {}).get("use") == "typed"),
                           "edge_in_cm": st.get("edge_in_cm"), "homes": st.get("homes"),
                           "keepouts": st.get("keepouts"), "guesses": st.get("guesses")}
            out["tracking_now"] = self.run_tracking()
            out["typed_words"] = list(getattr(self, "typed_words", []) or [])
            if self.last_production:
                out["production_orders"] = self.last_production.get("orders")
                out["production_choice"] = {k: self.last_production.get(k) for k in (
                    "experiment", "swap_halves", "sync_wiggle")}
            if self.program is not None and (self.program.get("production") or {}).get("experiment"):
                out["experiment_planned"] = self.program["production"]["experiment"]
        if getattr(self, "start", None) is not None:
            out["start"] = self.start.state()
        if self.feel is not None:
            out["feel"] = self.feel.status()
        if self.tag:
            out["tag"] = {k: v for k, v in self.tag.d.items()}
            out["tag"]["corner_clicks_on"] = self.tag.corner_clicks_on()
            out["tag"]["ref_cfg"] = self.tag.ref_cfg
            try:
                out["tag"]["refit_live"] = self.tag.refit_live(os.path.dirname(self.last_runs.get("mission") or ""))
            except Exception as e:     # noqa: BLE001  (a status line never breaks the page)
                out["tag"]["refit_live"] = {"state": "none", "why": str(e)}
            out["tag"]["room"] = bool(self.tag.room is not None and self.tag.room.poll() is None)
            out["sync"] = self.tag.sync_state()
        return out

    def conn_default(self):
        """The Connect screen's starting mode, worked out again every 5 s (the recorder may still be starting when
        the page first loads); the page follows it until the operator picks a mode."""
        t = time.time()
        if getattr(self, "_conn_default", None) is None or t - self._conn_default_t > 5.0:
            self._conn_default = conn_default(dry=True if getattr(self, "force_dry", False) else None)
            self._conn_default_t = t
        return self._conn_default

    # -- workspace ---------------------------------------------------------
    def set_map(self, d):
        m = copy.deepcopy(self.map)
        m["tasks"] = {}
        f0 = dict(m["floor"])
        old_start = {r["name"]: list(r["start"]) for r in m.get("robots", [])}
        for k in ("floor", "sides", "spots", "keepouts", "robots", "open_margin_cm", "notes"):
            if k in d:
                m[k] = d[k]
        f1 = m["floor"]
        if "floor" in d and any(abs(f0[k] - f1[k]) > 1e-6 for k in ("x_min", "y_min", "x_max", "y_max")):
            # a new table size: start spots the operator did not move go with the table (same share of it), kept
            # START_IN_CM inside (a corner start on a smaller table was left boxed in at its edge, 28 Oct screenshots)
            for r in m.get("robots", []):
                if [round(v, 6) for v in r["start"][:2]] != [round(v, 6) for v in old_start.get(r["name"], [None, None])[:2]]:
                    continue
                for j, (a, b) in enumerate((("x_min", "x_max"), ("y_min", "y_max"))):
                    share = (r["start"][j] - f0[a]) / max(1e-6, f0[b] - f0[a])
                    v = f1[a] + share * (f1[b] - f1[a])
                    r["start"][j] = round(min(f1[b] - START_IN_CM, max(f1[a] + START_IN_CM, v)), 1)
        self.map = FM.normalize(m)
        self.save_map()

    # -- warm-up -----------------------------------------------------------
    def warmup_plan(self):
        self.apply_warmup_knobs()
        self.warm_map = WU.plan_tasks(self.warm_source())
        over = dict(self.settings["planner"])
        MS.gap_lock_planner(over, n_robots=MS.mission_robots(self.warm_map))     # the 8 cm lock
        plan = P.make_plan(self.warm_map, over=over or None)
        with open(os.path.join(self.dir, "warm_map.json"), "w") as f:
            json.dump(self.warm_map, f, indent=1)
        P.save_plan(plan, os.path.join(self.dir, "warm_plan.json"))
        self.preview = preview(self.warm_map, plan, title="Warm-up")
        return {"preview": self.preview}

    def warm_source(self):
        """The map the warm-up plans on: only the robots that will drive (the ones setup heard, else the mission
        program's robots, else every robot in the map). A robot in the map but switched off (Center, the spare)
        made the robots' warm-up wait for it and end as "start-up failed" (30 Oct 09:37, Pi)."""
        want = list(getattr(self, "warm_robots", None) or [])
        if not want:
            prog = self.program if self.program is not None else MS.monday_program()
            want = list(prog.get("robots") or [])
        m = copy.deepcopy(self.map)
        keep = [r for r in m.get("robots", []) if r["name"] in want]
        if keep:
            m["robots"] = keep
        return m

    def warmup_analyze(self, folder=None):
        self.apply_warmup_knobs()
        folder = folder or self.last_runs.get("warmup")
        if not folder:
            raise ValueError("no warm-up run yet")
        import mapview as MV
        wm = self.warm_map or json.load(open(os.path.join(self.dir, "warm_map.json")))
        res = WU.analyze(wm, folder)
        res["words"] = WU.words(res)
        res["svg"] = MV.svg(FM.normalize(wm), res, MV.read_track(folder))
        res["folder"] = folder
        self.warm_result = res
        return res

    def warmup_apply(self, keep_zones=None, move_sides=False):
        if not self.warm_result:
            raise ValueError("analyze the warm-up first")
        self.map = WU.apply(self.map, self.warm_result, accept_zones=keep_zones, move_sides=move_sides)
        self.save_map()

    # -- missions ----------------------------------------------------------
    def make_mission(self, d):
        mode = d.get("mode", "monday")
        s = MS.settings(self.settings["mission"])
        m = FM.normalize(self.map)
        start = {r["name"]: r["start"] for r in m["robots"]}
        if mode == "program":
            # blocks (one system, 2026-10-28): Monday's default, or Nick's edit of it
            prog = d.get("program") or self.program or MS.monday_program()
            self.program = prog
            with open(os.path.join(self.dir, "program.json"), "w") as f:
                json.dump(prog, f, indent=1)
            mm = MS.build(m, prog, s)
        elif mode == "monday":
            mm = MS.monday(m, s, seed=int(d.get("seed", 1)))
        else:
            mm = dict(m)
            robot = d.get("robot") or "Center"
            if mode == "mow":
                v, notes = MS.mow(m, s, robot_start=start.get(robot))
                mm["tasks"] = {robot: {"visit": v, "park": True}}
            elif mode == "chicken":
                mm["tasks"] = {k: {"visit": v, "park": True} for k, v in MS.chicken_mission(m, s=s).items()}
            elif mode == "follow":
                mm["tasks"] = {k: {"visit": v, "park": True} for k, v in MS.follow(m, s=s).items()}
            elif mode == "roam":
                mm["tasks"] = {robot: {"visit": MS.roam(m, robot, s, seed=int(d.get("seed", 1)),
                                                        start=start.get(robot)), "park": True}}
            elif mode == "show":
                sh, end, figs = MS.show(m, s=s)
                mm["tasks"] = {k: {"visit": v, "park": True} for k, v in sh.items()}
            elif mode == "spots":
                lists = d.get("lists") or {}
                mm["tasks"] = {n: {"visit": MS.instructions(txt, m.get("spots") or {}), "park": True}
                               for n, txt in lists.items() if txt.strip()}
            else:
                raise ValueError("unknown mode %r" % mode)
            mm["mission"] = {"name": mode, "settings": s, "planner": {"separation_cm": s["separation_cm"]}}
        over = dict((mm.get("mission") or {}).get("planner") or {})
        over.update(self.settings["planner"])
        # the 8 cm lock: the planner screen's values never take a 2-robot mission under the net (plan_synced holds it
        # too; said here so the page shows it)
        said = MS.gap_lock_planner(over, MS.gap_floor(s), MS.mission_robots(mm), where="planner settings")
        if said:
            self.gap_notices = said
        plan = MS.plan_synced(mm, over=over, t_limit_s=float(d.get("t_limit_s", 60)))
        self.mission, self.plan = mm, plan
        with open(os.path.join(self.dir, "mission.json"), "w") as f:
            json.dump(mm, f, indent=1)
        P.save_plan(plan, os.path.join(self.dir, "plan.json"))
        self.preview = preview(mm, plan, title=(mm.get("mission") or {}).get("name", mode))
        return {"preview": self.preview, "gap_notices": list(self.gap_notices)}

    def default_program(self, robots=None):
        self.program = MS.monday_program(tuple(robots) if robots else ("Left", "Right"))
        return {"program": self.program}

    # -- runs --------------------------------------------------------------
    def start_run(self, d):
        if self.run and self.run.alive():
            raise ValueError("a run is going: stop it first")
        if getattr(self, "calib", None) and self.calib.running():
            raise ValueError("the calibration run is driving the robot: stop it first (Production screen)")
        self.stop_asked = False
        kind = d.get("kind", "mission")
        dry = bool(d.get("dry", True))
        conn = d.get("conn") or ("sim" if dry else "robots")
        if self.feel is not None and kind == "mission" and conn in ("sim-cameras", "robots") and \
                self.run_tracking() == "tags":
            # Feel out the table (Nick, 2 Nov 18:33): the first production run on a table (this camera setup) feels its
            # edges first, then starts by itself on the felt outline; later runs use the saved outline
            need, cal, why = self.feel.use_saved(conn)
            skip = bool(d.get("skip_feel")) or self.feel.skip_once
            self.feel.skip_once = False
            if need and not skip:
                if conn == "robots":
                    busy = self.others_driving()
                    if busy:
                        raise ValueError("%s is driving the robots on this Pi: a robot run can start when it has ended "
                                         "(the Live lab tab shows it and stops it)." % busy)
                st = self.feel.start(conn, chain=dict(d, conn=conn))
                return dict(self.run.status() if self.run else {}, feel=st)
            if need and skip:
                self.feel.words = ["Skipped this time (you chose it): the production run uses the cameras' outline "
                                   "as it is. %s" % why]
                self.feel.phase = "skipped"
        if conn in ("sim-cameras", "robots"):
            return self.start_cam_run(kind, conn, d)
        rs = dict(self.settings["run"])
        rs.update(d.get("run") or {})
        if kind == "warmup":
            if not self.warm_map:
                self.warmup_plan()
            mp, pp = os.path.join(self.dir, "warm_map.json"), os.path.join(self.dir, "warm_plan.json")
            border = 0.0                       # the feels go to the edge on purpose (the edge guard stops them)
        else:
            if not self.mission:
                raise ValueError("make a mission first")
            mp, pp = os.path.join(self.dir, "mission.json"), os.path.join(self.dir, "plan.json")
            border = float(rs["border_cm"])
        folder = os.path.join(self.dir, "runs", "%s-%s%s" % (now_tag(), kind, "-sim" if dry else ""))
        cmd = [sys.executable, os.path.join(HERE, "run_plan.py"), "--map", mp, "--plan", pp, "--yes",
               "--session", folder, "--timeout-s", str(rs["timeout_s"]), "--border-cm", str(border),
               "--improve-budget-s", str(rs["improve_budget_s"])]
        for c in [x.strip() for x in str(rs.get("edge_guard", "")).split(";") if x.strip()]:
            cmd += ["--send", c]
        if rs.get("turn_pause", True) and kind != "warmup":
            cmd += ["--turn-pause"]
        if not rs.get("mirror", True):
            cmd += ["--no-mirror"]
        cmd += ["--on-problem", self.on_problem()]          # keep going or stop (2026-11-03)
        if dry:
            cmd += ["--dry-run"]
            if rs.get("sim_true_sides"):
                cmd += ["--sim-arg=--true-sides=%s" % rs["sim_true_sides"]]
            for o in [x.strip() for x in str(rs.get("sim_true_objects", "")).split(";") if x.strip()]:
                cmd += ["--sim-arg=--true-object=%s" % o]
            if rs.get("sim_brick") and kind != "warmup":
                cmd += ["--brick", rs["sim_brick"]]
                if rs.get("sim_brick_camera"):
                    cmd += ["--sim-arg=--brick-camera"]
        self.run = Run(kind, folder, cmd, env=self.safety_env())
        self.last_runs[kind] = folder
        self.run.monitor = monitor_for(folder, self.run.p.pid)          # (12 Nov) pi_monitor.csv
        return self.run.status()

    def run_tracking(self):
        """tags / lights / none for the runs from this page: the Run screen's selector, else the site's run_tracking."""
        v = (self.settings.get("run") or {}).get("tracking") or ""
        if v in ("tags", "lights", "none"):
            return v
        return (self.site or {}).get("run_tracking") or (self.site or {}).get("tracking") or "lights"

    def on_problem(self):
        """keep_going or stop for the runs from this page: the Run screen's tick-box, else the site's (e08: keep_going everywhere)."""
        v = (self.settings.get("run") or {}).get("on_problem") or ""
        if v in ("keep_going", "stop"):
            return v
        return (self.site or {}).get("on_problem") or "keep_going"      # (e08, Q7) keep going by default

    def start_cam_run(self, kind, conn, d):
        """The run through the cameras (mission_run.py): the robots on the Pi, or the simulator's two cameras through
        the same camera bridge (the chain the Pi runs). Mission Control's run and `./run-fleet-pi.sh monday` are then
        the same program."""
        rs = dict(self.settings["run"])
        rs.update(d.get("run") or {})
        if conn == "robots":
            busy = self.others_driving()
            if busy:
                raise ValueError("%s is driving the robots on this Pi: a robot run can start when it has ended (the "
                                 "Live lab tab shows it and stops it). The simulator can run meanwhile." % busy)
        folder = os.path.join(self.dir, "runs", "%s-%s-%s" % (now_tag(), kind, conn))
        cmd = [sys.executable, os.path.join(HERE, "mission_run.py"), "--session", folder,
               "--timeout-s", str(rs["timeout_s"]), "--border-cm", str(rs["border_cm"])]
        trk = self.run_tracking() if self.tag else None
        if self.tag and trk != "tags":
            # with or without tags (2026-11-03): the same site, followed by the robots' lights (camera bridge, corners
            # clicked) or by nothing (odometry only); no feel-out, no refit (both need the roof tags)
            self.tag.stop_room()
            cmd += ["--tracking", trk, "--site", self.site["name"], "--session-id",
                    self.tag.sync_state()["session_id"], "--feel-table", "skip"]
        elif self.tag:
            # the AprilTag site (2026-11-02): the practice room closes first (its simulator would answer as the robots),
            # the run reads the roof tags, writes the sync marks with the session id, and refits the cameras after
            self.tag.stop_room()
            cmd += ["--tracking", "tags", "--site", self.site["name"], "--session-id",
                    self.tag.sync_state()["session_id"]]
            if kind != "feel":
                cmd += ["--feel-table", "skip"]                # Mission Control felt the table first (feel_job.py)
                # the refit after the run (2026-11-03, risk review #10): per site, "background" (the default: its own
                # process, the run's end and this screen never wait), "wait" (the old way: the run ends after it) or
                # "off" (the Refit button only)
                ar = str(self.site.get("auto_refit", "background")).lower()
                if ar in ("wait", "old", "blocking"):
                    cmd += ["--refit"]
                elif ar not in ("off", "no", "false", "button"):
                    cmd += ["--refit-bg"]
            if conn == "sim-cameras" and rs.get("sim_true_floor"):
                cmd += ["--sim-true-floor", str(rs["sim_true_floor"])]
        if kind == "feel":
            # Feel out the table (feel_job.py): the feels as a made mission; every robot calibrated first; the robots'
            # own border off (the crawls go to the edge on purpose; the edge guard stays on); no turn pauses
            import feel_table as FTB
            fmp, fpp = self.feel_paths
            folder = d.get("folder") or folder
            cmd[cmd.index("--session") + 1] = folder
            cmd += ["--mission", fmp, "--plan", fpp, "--border-cm", "0", "--feel-table", "skip"]
            if d.get("calibrate"):
                cmd += FTB.CALIBRATE_ARGS          # the surface check's run: every robot fully calibrated first
        elif kind == "warmup":
            if not self.warm_map:
                self.warmup_plan()
            cmd += ["--mission", os.path.join(self.dir, "warm_map.json"), "--plan", os.path.join(self.dir, "warm_plan.json"),
                    "--border-cm", "0"]
        elif kind == "visit":
            # (11-05h, Nick 5 Nov 20:28) Visit points, one button, no settings: every robot on the table found by the
            # wiggle (however many), each tag's heading checked by a nudge, the points shared over the area the camera
            # reads (skipped points listed with a reason), straight lines with turns on the spot
            if not self.tag:
                raise ValueError("Visit points needs a site with roof tags on the robots")
            cmd += ["--robots", "auto", "--split", "auto", "--motion", "stopturn"]
            if str(d.get("order") or "").strip():
                # (e11, Nick 9 Nov) the points in the order Nick chose (typed, map or webcam clicks); unreadable points
                # skipped unless he keeps them
                cmd += ["--visit-order", str(d["order"]).strip()]
                if d.get("keep_unreadable"):
                    cmd += ["--visit-keep-unreadable"]
            if conn == "sim-cameras" and "--sim-robots" not in (d.get("extra") or []):
                cmd += ["--sim-robots", str(int(d.get("sim_robots") or 2)), "--sim-bind"]
            cmd += list(d.get("extra") or [])
        elif kind == "roam":
            # (e07, Nick 6 Nov) the Start screen's Free roam at a roof-tag site: every robot found by its wiggle
            # wanders over its share of the readable points in a random order, on curves (firmware 11-05c+)
            if not self.tag:
                raise ValueError("free roam through mission_run is for a roof-tag site (home: freeroam.py)")
            rm = float(d.get("roam_min") or 10.0)
            cmd[cmd.index("--timeout-s") + 1] = str(rm * 60.0 + 240.0)
            cmd += ["--robots", "auto", "--split", "auto", "--motion", d.get("motion") or "curves", "--roam-min", "%g" % rm]   # (Nick 9 Nov) Advanced: free roam's driving
            cmd += list(d.get("extra") or [])
        elif kind == "experiment":
            # (e07) the Start screen's Experiment routes: Jay-Sun's next route for every robot found by its wiggle
            # (two robots: his halves exactly; another count: the table cut into that many runs), straight lines
            if not self.tag:
                raise ValueError("experiment routes need a site with roof tags and typed points")
            ex_ = str(d.get("experiment") or "e1")
            # the shakedown runs with no camera (robots placed on their homes by hand): the site's own robots, no wiggle
            rob_ = ",".join(r["name"] for r in self.site["robots"]) if ex_ == "shakedown" else "auto"
            cmd += ["--robots", rob_, "--experiment", ex_, "--swap-halves", "auto", "--motion", "stopturn"]
            if d.get("allow_half_table"):
                cmd += ["--allow-half-table"]
            cmd[cmd.index("--timeout-s") + 1] = str(max(float(rs["timeout_s"]), 1500.0))
            cmd += list(d.get("extra") or [])
        else:
            if self.program is None:
                self.program = MS.monday_program()
            if self.tag and conn == "robots":
                w_ = self.experiment_auto_swap()
                if w_:
                    print(w_, flush=True)
            pp = os.path.join(self.dir, "program.json")
            with open(pp, "w") as f:
                json.dump(self.program, f, indent=1)
            cmd += ["--map", self.map_path, "--program", pp, "--robots", ",".join(self.program.get("robots", []))]
        if not rs.get("turn_pause", True) or kind in ("warmup", "feel", "roam"):
            cmd += ["--no-turn-pause"]
        if not rs.get("straighten", True):
            # the warm-up straightens too (Nick, 2026-10-30 09:50: "they should be able to self-correct"): its facing
            # check stopped a real warm-up over Left facing backwards and Right sideways; now each is turned or driven
            # to its start like Monday's run
            cmd += ["--no-straighten"]
        if not rs.get("mirror", True):
            cmd += ["--no-mirror"]
        cmd += ["--on-problem", self.on_problem()]          # the Run screen's tick-box, else the site's (2026-11-03)
        if conn == "sim-cameras":
            cmd += ["--dry"]
            if rs.get("sim_washout"):
                cmd += ["--washout"]
            if float(rs.get("sim_drop_pct") or 0):
                cmd += ["--drop-pct", str(rs["sim_drop_pct"])]
            if rs.get("sim_dead"):
                cmd += ["--dead", str(rs["sim_dead"])]
            for a in rs.get("sim_args") or []:      # the simulator's own faults (tests: robots put down anywhere, ...)
                cmd += ["--sim-arg=%s" % a]
        else:
            if rs.get("farm"):
                cmd += ["--farm"]
            if rs.get("upload"):
                cmd += ["--upload"]
        self.run = Run(kind, folder, cmd, env=self.safety_env())
        self.run.session = os.path.join(folder, "run")
        self.run.mode = conn
        self.run.monitor = monitor_for(folder, self.run.p.pid)          # (12 Nov) pi_monitor.csv in the run folder
        self.last_runs[kind] = os.path.join(folder, "run")
        return self.run.status()

    def production(self, d):
        """The Production screen's orders -> a program -> planned like any program (the 8 cm net, the preview)."""
        if not self.tag:
            raise ValueError("production runs are for a site with tags on the robots")
        prog = self.tag.production_program(d)
        out = self.make_mission({"mode": "program", "program": prog})
        out["program"] = prog
        self.last_production = dict(d)
        ex = (prog.get("production") or {}).get("experiment")
        if ex:
            # the shakedown is the no-camera run (Jay-Sun, 3 Nov); an experiment after it goes back to the site's tracking
            rs = self.settings.setdefault("run", {})
            if ex["id"] == "shakedown" and rs.get("tracking") != "none":
                rs["tracking"] = "none"
                out["words"] = "Shakedown: tracking set to No camera (odometry only) on the Run screen"
            elif ex["id"] != "shakedown" and rs.get("tracking") == "none" and d.get("experiment") != "custom":
                rs["tracking"] = ""
                out["words"] = "Tracking back to the site's (%s) on the Run screen" % self.run_tracking()
            self.save_settings() if hasattr(self, "save_settings") else None
        return out

    def experiment_auto_swap(self):
        """Before a production run (2026-11-03): an experiment planned with swap auto is planned again if the run
        counter now says the other way (the counter counts each run started on the robots). Returns words or None."""
        lp = self.last_production or {}
        ex = ((self.program or {}).get("production") or {}).get("experiment")
        if not ex or str(lp.get("swap_halves") or "auto") != "auto" or ex.get("id") == "shakedown":
            return None
        import experiments as EX
        sw, words = EX.swap_for(self.site, "auto")
        if bool(sw) == bool(ex.get("swap_halves")):
            return None
        self.production(lp)
        return "Planned again for this run: %s" % words

    def replan_production(self):
        """The production mission planned again on the map as it is now (the feel-out moved the outline: homes and
        points follow it). Nick's last orders; else the program in use."""
        if self.last_production:
            return self.production(self.last_production)
        if self.program is not None:
            return self.make_mission({"mode": "program", "program": self.program})
        return None

    def others_driving(self):
        """What else drives the robots on this Pi (an E02 session, the DOE loop, Monday's mission from a terminal, the
        ladder), in words, or "" (the start page's process scan, tools/pi/launcher.py; simulator runs do not count).
        Two programs sending the same robots moves would fight; Mission Control's own run is checked by start_run."""
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
            import launcher as LA
            p = self.proc_scan() if getattr(self, "proc_scan", None) else LA.scan_procs()
        except Exception:     # noqa: BLE001  (no /proc: nothing known to be running)
            return ""
        dry = set(p.get("dry") or [])
        words = []
        if p.get("runner"):
            words.append("An E02 session")
        for k, w in (("doe", "The DOE loop"), ("mission", "Monday's mission"), ("ladder", "The ladder"),
                     ("plan_run", "A planner test (run_plan)")):
            if any(x not in dry for x in p.get(k) or []):
                words.append(w)
        return " and ".join(words)

    def frame(self, cam="overhead"):
        """The newest picture of a camera with the robots ringed and the planned paths drawn (the Run screen):
        the recorder's /live on the Pi, or the simulator's cameras."""
        import cv2
        import numpy as np
        run = self.run
        mode = getattr(run, "mode", "sim") if run else "sim"
        if self.tag is not None:
            # the AprilTag site (2026-11-02): the run's own fitted cameras (simulator) or the site's (robots)
            port = (8765 if cam == "overhead" else 8766) if mode == "sim-cameras" else (8000 if cam == "overhead" else 8001)
            base = os.path.join(os.path.dirname(run.session), "tagcal") if (run is not None and mode == "sim-cameras") \
                else self.tag.cal_dir
            cal = os.path.join(base, cam + ".json")
        elif mode == "sim-cameras":
            port = 8765 if cam == "overhead" else 8766
            cal = os.path.join(run.session, "sim", "table_camera.json" if cam == "overhead" else "side_camera.json")
        else:
            port = 8000 if cam == "overhead" else 8001
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
            import setup_conf as SCF
            cal = SCF.overhead_calib(relative=False) if cam == "overhead" else (SCF.side_calib() or None)
            if isinstance(cal, tuple):
                cal = cal[0]
        jpg = grab_jpeg("127.0.0.1", port)
        if not hasattr(self, "last_frames"):
            self.last_frames = {}
        if jpg is None:
            # the run ended (the simulator's cameras gone, or the recorder stopped): its last ringed picture
            return self.last_frames.get(cam)
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "experiment"))
            import e02_camera as CAM
            model = CAM.load_camera(cal) if cal and os.path.exists(cal) else None
        except Exception:     # noqa: BLE001
            model = None
        if model is not None and run is not None:
            col = {"Left": (204, 82, 41), "Center": (12, 65, 194), "Right": (200, 62, 122)}      # BGR of the page's colours
            for r_ in (self.site or {}).get("robots", []):
                h_ = r_["colour"].lstrip("#")
                col.setdefault(r_["name"], (int(h_[4:6], 16), int(h_[2:4], 16), int(h_[0:2], 16)))
            plan_p = os.path.join(os.path.dirname(run.session), "plan.json") if getattr(run, "session", None) else None
            if plan_p and os.path.exists(plan_p):
                with open(plan_p) as f:
                    plan = json.load(f)
                for n, r in plan["robots"].items():
                    pts = [r["start"][:2]] + [st["to"][:2] for st in r["steps"] if st["type"] in ("drive", "arc")]
                    px = [model.project((x, y, 0.0)) for x, y in pts]
                    px = [(int(u), int(v)) for u, v in px if np.isfinite(u) and np.isfinite(v)]
                    if len(px) > 1:
                        cv2.polylines(img, [np.array(px, np.int32)], False, col.get(n, (120, 120, 120)), 1, cv2.LINE_AA)
            live = (run.status().get("live") or {}).get("robots", {})
            for n, r in live.items():
                p = r.get("cam") or r.get("own")
                if not p:
                    continue
                zr = float(((self.site or {}).get("tags") or {}).get("height_cm") or 6.0) if self.tag else \
                    (4.0 if cam == "overhead" else 1.85)
                u, v = model.project((p[0], p[1], zr))
                if np.isfinite(u) and np.isfinite(v):
                    rr = int(max(8, 6.5 * model.px_per_cm(p[0], p[1], 4.0)))
                    cv2.circle(img, (int(u), int(v)), rr, col.get(n, (0, 200, 255)), 2, cv2.LINE_AA)
                    cv2.putText(img, n, (int(u) + rr + 2, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col.get(n, (0, 200, 255)),
                                2, cv2.LINE_AA)
        ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.last_frames[cam] = out.tobytes()
        return out.tobytes() if ok else None

    def lighting(self):
        """The lighting check (lighting.py) on each camera's newest picture: does each camera see the robots against
        the board? Plain words per camera ("overhead camera washed out: ... close the blinds ..."), and the
        recorder's exposure changes (camrec WashoutGuard) since it started."""
        import cv2
        import numpy as np
        import lighting as LG
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "experiment"))
        import e02_camera as CAM
        run = self.run
        sim = getattr(run, "mode", "") == "sim-cameras" if run else False
        out = dict(cameras=[], exposure=[], t=round(time.time(), 1))
        n_rob = len((self.map or {}).get("robots") or []) or None
        for cam in ("overhead", "side"):
            if sim:
                port = 8765 if cam == "overhead" else 8766
                cal = os.path.join(run.session, "sim", "table_camera.json" if cam == "overhead" else "side_camera.json")
            else:
                port = 8000 if cam == "overhead" else 8001
                sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
                import setup_conf as SCF
                cal = SCF.overhead_calib(relative=False) if cam == "overhead" else (SCF.side_calib() or None)
                if isinstance(cal, tuple):
                    cal = cal[0]
            jpg = grab_jpeg("127.0.0.1", port)
            if jpg is None or not cal or not os.path.exists(cal):
                out["cameras"].append(dict(camera=cam, verdict="no picture", words="%s camera: no picture (%s)" % (
                    cam, "is the recorder running?" if jpg is None else "no calibration saved")))
                continue
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            m = LG.measure(img, CAM.load_camera(cal), z=CAM.ROBOT_TOP_CM if cam == "overhead" else
                           0.5 * CAM.ROBOT_TOP_CM, name=cam, n_robots=n_rob)
            out["cameras"].append(m)
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pi"))
            import camrec as CR
            with open(CR.EXPOSURE_LOG) as f:
                out["exposure"] = [ln.strip().split(",", 1) for ln in f.readlines()[-5:]]
        except Exception:      # noqa: BLE001  (no recorder change yet)
            pass
        self.last_lighting = out
        return out

    def mapping(self):
        """The engineer's mapping view for the current or last run: each robot's camera track in its colour over the
        table, contacts from the warm-up if there were any (mapview.py)."""
        import mapview as MV
        run = self.run
        folder = getattr(run, "session", None) or (run.folder if run else None) or self.last_runs.get("mission")
        if not folder:
            raise ValueError("no run yet")
        m = FM.normalize(self.mission or self.map)
        return {"svg": MV.svg(m, self.warm_result or {}, MV.read_track(folder)), "folder": folder}

    # -- one-button setup (Nick 12:56 and 18:36): the flash check, the chessboard corner piece, then the warm-up -----
    SETUP_PORT = 8096

    def setup_start(self, d):
        """Runs the camera setup page (tools/pi/camera_setup.py; --fake in the simulator) headless: its flash check
        first (side camera delay, which lights are which robot), then the chessboard in the 0,0 corner for both
        cameras' poses, lens heights and 0,0, saved as the calibrations in use; then the warm-up (Nick confirms its
        ~15 pictures in 90 s on the Warm-up screen). Nothing is locked: each part can also be run alone on the setup
        page (port 8096)."""
        if getattr(self, "setup_job", None) and self.setup_job.get("running"):
            raise ValueError("the setup is already running")
        fake = d.get("conn", "sim") != "robots"
        table = self.surface(d.get("surface"))
        job = {"running": True, "steps": [], "fake": fake, "error": None}
        self.setup_job = job
        threading.Thread(target=self._setup, args=(job, fake, table, d), daemon=True).start()
        return self.setup_state()

    def surface(self, given=None):
        """The surface whose 4 corners the cameras see, "LxW" cm, long side along x (session plan 2026-10-28): at home
        the 122 x 122 plywood, with Monday's smaller table marked on it from the 0,0 corner; at school the table
        itself. The chessboard solve pairs these corners with this size, so it must be the surface's, not the
        mission table's (a 122 x 61 table with the plywood's 4 corners gave a wrong or refused calibration). Given:
        saved in the map ("surface_cm") for next time; else the saved one; else the table's size."""
        if given:
            a, b = (float(v) for v in str(given).lower().replace(" ", "").split("x"))
            if min(a, b) <= 0:
                raise ValueError("the surface's size must be two lengths in cm, e.g. 122x122")
            self.map["surface_cm"] = [max(a, b), min(a, b)]
            self.save_map()
        sc = self.map.get("surface_cm")
        if sc:
            return "%gx%g" % (sc[0], sc[1])
        f = self.map["floor"]
        return "%gx%g" % (f["x_max"] - f["x_min"], f["y_max"] - f["y_min"])

    def setup_state(self):
        j = getattr(self, "setup_job", None) or {}
        return {k: j.get(k) for k in ("running", "steps", "fake", "error", "checker", "flash", "wait_board")}

    def setup_board_away(self):
        """Nick took the chessboard away: the setup goes on to the warm-up."""
        j = getattr(self, "setup_job", None)
        if not j or not j.get("wait_board"):
            raise ValueError("the setup is not waiting for the chessboard")
        j["board_away"] = True
        return self.setup_state()

    def _setup(self, job, fake, table, d):
        import urllib.request

        def call(path, body=None, timeout=60):
            req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.SETUP_PORT, path),
                                         data=None if body is None else json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())

        def step(text):
            job["steps"].append([time.strftime("%H:%M:%S"), text])
        page = None
        # the old chessboard way opens the cameras itself: the recorder is stopped first and started again after (the
        # warm-up and the run read its pictures; Nick's run sheet found the clash, 29 Oct)
        rec_was = (not fake) and SJ.stop_recorder(say=step)
        try:
            cmd = [sys.executable, os.path.join(os.path.dirname(HERE), "pi", "camera_setup.py"), "--port",
                   str(self.SETUP_PORT), "--out", os.path.join(self.dir, "setup-" + now_tag())]
            if fake:
                cmd.append("--fake")
            step("camera setup page starting%s" % (" (simulated cameras)" if fake else ""))
            page = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            for _ in range(60):
                try:
                    call("/api/state", timeout=2)
                    break
                except OSError:
                    time.sleep(0.5)
            step("1. the overhead picture's table corners")
            call("/api/grab", {})
            step("2. the flash check: the side camera's delay, each robot's lights")
            call("/api/flash", {})
            for _ in range(240):
                st = call("/api/state")
                if st["doe"]["state"] != "running":
                    break
                time.sleep(1.0)
            job["flash"] = {"state": st["doe"]["state"], "words": st["doe"].get("words"), "lag": st["doe"].get("lag")}
            step("   " + (st["doe"].get("words") or st["doe"]["state"])[:200])
            if st.get("side"):
                call("/api/side_grab", {})
            step("3. the chessboard in the 0,0 corner: both cameras' poses and lens heights")
            st = call("/api/checker_save", {"table": table}, timeout=120)
            job["checker"] = st.get("checker")
            for which, r in (st.get("checker") or {}).items():
                step("   %s: %s" % (which, ("lens %.1f cm up, board fit %.2f px, table corners within %.1f px, saved"
                                           % (r["height_cm"], r["board_err_px"], r["table_err_px"])) if r.get("ok")
                                  else r.get("why")))
        except Exception as e:     # noqa: BLE001
            job["error"] = str(e)
            step("stopped: %s" % e)
        finally:
            if page is not None:
                page.send_signal(signal.SIGINT)
                try:
                    page.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    page.kill()
            if not fake and (rec_was or d.get("warmup", True)):
                ok, w = SJ.start_recorder(say=step)
                step("   " + w)
        if not job["error"] and d.get("warmup", True) and not fake:
            # the robots would drive over the chessboard: the warm-up waits for Nick to take it away (10 min at most)
            step("   take the chessboard away, then press \"Chessboard taken away\"")
            job["wait_board"] = True
            for _ in range(1200):
                if job.get("board_away"):
                    break
                time.sleep(0.5)
            job["wait_board"] = False
            if not job.get("board_away"):
                job["error"] = "the chessboard was not taken away within 10 minutes: warm-up not started"
                step(job["error"])
        if not job["error"] and d.get("warmup", True):
            step("4. the warm-up (corners and sides); then check its pictures on the Warm-up screen")
            try:
                with self.lock:
                    self.warmup_plan()
                    self.start_run({"kind": "warmup", "conn": d.get("run_conn") or ("sim" if fake else "robots"),
                                    "dry": fake})
            except Exception as e:     # noqa: BLE001
                job["error"] = str(e)
                step("warm-up not started: %s" % e)
        job["running"] = False

    def pause(self, on):
        if not self.run or not self.run.alive():
            raise ValueError("nothing is running")
        self.run.signal(signal.SIGUSR1 if on else signal.SIGUSR2)
        self.run.paused = bool(on)
        return self.run.status()

    def stop_run(self):
        self.stop_asked = True       # (2026-11-03) keep going never starts the next run of a chain after a Stop
        if getattr(self, "calib", None) and self.calib.running():
            self.calib.stop()        # (2026-11-04) the calibration run stops too (stopall's Mission Control step)
        if self.run:
            self.run.stop()
            return self.run.status()
        return None

    def review(self, folder=None):
        folder = folder or self.last_runs.get("mission") or (self.run.folder if self.run else None)
        if not folder:
            raise ValueError("no run yet")
        out = {"folder": folder}
        for name in ("run_plan_summary.json", os.path.join("sim", "sim_report.json")):
            p = os.path.join(folder, name)
            if os.path.exists(p):
                with open(p) as f:
                    out[os.path.basename(name).split(".")[0]] = json.load(f)
        import mapview as MV
        out["tracks"] = {n: pts[::3] for n, pts in MV.read_track(folder).items()}
        return out


def grab_jpeg(host, port, timeout=2.0):
    """One JPEG from a /live multipart stream (the recorder's, or the simulator's), or None."""
    import socket as _s
    try:
        c = _s.create_connection((host, port), timeout=timeout)
    except OSError:
        return None
    try:
        c.sendall(b"GET /live HTTP/1.0\r\nHost: x\r\n\r\n")
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = c.recv(65536)
            if not chunk:
                break
            buf += chunk
            a = buf.find(b"\xff\xd8")
            b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
            if a >= 0 and b > a:
                return buf[a:b + 2]
        return None
    except OSError:
        return None
    finally:
        c.close()


def preview(mm, plan, title=""):
    """The plan sampled every PREVIEW_STEP_S for the page's time slider, with its summary numbers."""
    T = float(plan["summary"]["makespan_s"])
    n = int(T / PREVIEW_STEP_S) + 2
    frames = {}
    for name in plan["robots"]:
        frames[name] = [[round(v, 1) for v in P.pose_at(plan, name, min(T, k * PREVIEW_STEP_S))[:2]] +
                        [round(math.degrees(P.pose_at(plan, name, min(T, k * PREVIEW_STEP_S))[2]), 0)]
                        for k in range(n)]
    acts = []
    for name, r in plan["robots"].items():
        cur = None
        for st in r["steps"]:
            lab = (st.get("label") or "").split(" lane")[0].split(" ready")[0].split(" swerve")[0].split(" sidestep")[0].split(" pass")[0].split(" aside")[0]
            if lab and lab != cur:
                acts.append({"robot": name, "label": lab, "t": st["t_start"]})
                cur = lab
    paths = {}
    for name, r in plan["robots"].items():
        pts = [[round(v, 1) for v in r["start"][:2]]]
        for st in r["steps"]:
            if st["type"] in ("drive", "approach"):
                pts.append([round(st["to"][0], 1), round(st["to"][1], 1)])
            elif st["type"] == "arc":           # a true curve: drawn through points along it
                pts += [[round(q[0], 1), round(q[1], 1)] for q in P.step_poses(st, 6)[1:]]
        paths[name] = pts
    starts = {name: r["start"][:2] for name, r in plan["robots"].items()}
    sm = MS.summary(mm, plan)
    return {"title": title, "step_s": PREVIEW_STEP_S, "makespan_s": T, "frames": frames, "paths": paths,
            "starts": starts, "acts": acts, "segs": block_segments(plan), "summary": sm, "closest_pair": plan["summary"].get("closest_pair"),
            "warnings": plan.get("warnings", []), "planning_time_s": plan["summary"].get("planning_time_s"),
            "notes": (mm.get("mission") or {}).get("notes", []), "figures": (mm.get("mission") or {}).get("figures", []),
            "floor": mm["floor"], "obstacles": mm.get("obstacles", []), "keepouts": mm.get("keepouts", [])}


def block_of(label):
    """A step's label -> the routine's part it belongs to ("mow lane 3 end" -> mow, "chicken 1 swerve" -> chicken)."""
    lab = (label or "").strip().lower()
    for head in ("mow", "chicken", "meet", "circle", "spread out", "dance", "roam", "follow"):
        if lab.startswith(head):
            return head
    for tail in (" ready", " swerve", " sidestep", " pass", " aside", " start", " end"):
        if lab.endswith(tail):
            lab = lab[: -len(tail)]
    return lab or "move"


def block_segments(plan):
    """Each robot's route cut into the routine's parts (Nick 2026-10-28: the Chicken part was lost in one line): per
    robot [{block, t0, t1, pts}], in order. A step with no label belongs to the part its next labelled step reaches
    (the drive towards a lane's start is mowing); a part met again later is "<part> on" (mow, then mow on)."""
    out = {}
    for name, r in plan["robots"].items():
        steps = r["steps"]
        nxt, labs = None, [None] * len(steps)
        for i in range(len(steps) - 1, -1, -1):
            if steps[i].get("label"):
                nxt = steps[i]["label"]
            labs[i] = nxt
        prev = None
        for i in range(len(steps)):          # trailing unlabelled steps (the park's approach): the part before
            if labs[i] is None:
                labs[i] = prev
            prev = labs[i]
        cur = [round(v, 1) for v in r["start"][:2]]
        segs, seen = [], {}
        for st, lab in zip(steps, labs):
            b = block_of(lab)
            if not segs or segs[-1]["kind"] != b:
                seen[b] = seen.get(b, 0) + 1
                nm = b if seen[b] == 1 else ("%s on" % b if seen[b] == 2 else "%s %d" % (b, seen[b]))
                segs.append({"block": nm, "kind": b, "t0": round(st["t_start"], 2), "t1": round(st["t_start"], 2),
                             "pts": [list(cur)]})
            sg = segs[-1]
            sg["t1"] = round(st.get("t_end", st["t_start"] + st.get("duration_s", 0.0)), 2)
            if st["type"] in ("drive", "approach"):
                cur = [round(st["to"][0], 1), round(st["to"][1], 1)]
                sg["pts"].append(cur)
            elif st["type"] == "arc":
                pts = [[round(q[0], 1), round(q[1], 1)] for q in P.step_poses(st, 6)[1:]]
                sg["pts"] += pts
                cur = pts[-1] if pts else cur
        for sg in segs:
            sg.pop("kind")
        out[name] = segs
    return out


def clean(o):
    """JSON for the browser: infinities and NaN (e.g. 'closest to a board: none') become null."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    return o


def idle_sleep_due(busy, last_busy_t, now_t, slept, idle_min):
    """(11-05h) True when the robots should be put to sleep: nothing has driven them for idle_min minutes since the
    last run ended (and they were not put to sleep since). idle_min 0 = never."""
    return bool(idle_min) and not busy and last_busy_t is not None and not slept and \
        now_t - last_busy_t >= idle_min * 60.0


def idle_sleeper(s, every_s=30.0):
    """(11-05h, Observe then trust: robot state) Runs stay awake at their end (a wake restarts a robot and wipes values
    not saved on it); Mission Control puts every robot to sleep (`sleep 10`, the WiFi broadcast) after
    observe.IDLE_SLEEP_MIN minutes with no run, so the battery is not drained overnight. The site's idle_sleep_min
    changes the wait (0: never)."""
    import observe as OB
    last, slept = None, True
    while True:
        try:
            st_ = getattr(s, "start", None)
            if st_ is not None:
                st_.status()            # (e07) a run that ended: its robots sleep right away (Advanced switch)
            busy = bool((s.run and s.run.alive()) or (getattr(s, "calib", None) and s.calib.running()) or
                        s.others_driving() or (st_ is not None and st_.busy()))
            if busy:
                last, slept = time.time(), False
            idle = float((s.site or {}).get("idle_sleep_min", OB.IDLE_SLEEP_MIN)) if getattr(s, "site", None) else \
                OB.IDLE_SLEEP_MIN
            if st_ is not None:
                # (e07, Nick 6 Nov) the 30-minute timer is a switch in Advanced, off by default (sleep when done is on)
                idle = float(st_.adv.get("sleep_timer_min") or 30.0) if st_.adv.get("sleep_timer") else 0.0
                if st_.active_practice and not busy:
                    slept = True        # a practice run moved no robot
            every_s = 5.0
            if idle_sleep_due(busy, last, time.time(), slept, idle):
                ck = int((s.site or {}).get("sleep_checkin_min", 10)) if getattr(s, "site", None) else 10
                subprocess.run([sys.executable, os.path.join(os.path.dirname(HERE), "pi", "robot_sleep.py"), "sleep",
                                str(ck), "255.255.255.255"], timeout=20, capture_output=True)
                print("%.0f minutes with no run: every robot sent `sleep %d` (it wakes for 20 s every %d min; "
                      "robot_sleep.py wake any brings it back)" % (idle, ck, ck), flush=True)
                slept = True
        except Exception as e:  # noqa: BLE001  (the sleeper never stops Mission Control)
            print("(idle sleeper: %s)" % e, flush=True)
        time.sleep(every_s)


class Handler(BaseHTTPRequestHandler):
    session = None

    def log_message(self, *a):
        pass

    def _send(self, code, obj, ctype="application/json"):
        body = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(clean(obj)).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        S = self.session
        try:
            if path in ("/", "/index.html", "/classic", "/table"):
                # (Mission Control build, Nick 12 Nov) the new map-first page ("the table") at /; the old page kept at
                # /classic, and at / again when Settings > "Open the old page by default" (or MIRLT_MC_PAGE=classic)
                with open(page_for(path, S), "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                with S.lock:
                    return self._send(200, S.state())
            if path == "/api/run":
                return self._send(200, S.run.status() if S.run else {})
            if path == "/api/start":
                return self._send(200, clean(S.start.state()))
            if path == "/api/pi_health":               # (12 Nov, Part 3) the Pi health panel
                return self._send(200, clean(pi_health_state(S)))
            if path == "/api/live":                    # (12 Nov, 1c) the live table: robots, their doubt, the points
                import live_view as LV
                return self._send(200, clean(LV.snapshot(S.start)))
            if path == "/api/calib":
                return self._send(200, S.calib.status())
            if path == "/api/calib/file":
                from urllib.parse import parse_qs
                nm = (parse_qs(urlparse(self.path).query).get("name") or [""])[0]
                body = S.calib.file(nm)
                return self._send(200, body, "image/png" if nm.endswith(".png") else
                                  "text/plain; charset=utf-8" if nm.endswith(".md") else "application/json")
            if path == "/api/review":
                with S.lock:
                    return self._send(200, S.review())
            if path == "/api/visit/points":
                # (e11, Nick 9 Nov) the order picker: stickers on the map and in the camera's picture
                from urllib.parse import parse_qs
                cam = (parse_qs(urlparse(self.path).query).get("cam") or ["overhead"])[0]
                if not S.tag:
                    return self._send(404, {"error": "Visit points in an order needs a site with roof tags"})
                return self._send(200, clean(S.start.visit_points(cam)))
            if path == "/api/place":                   # (e13) held robots waiting for a person's click
                return self._send(200, clean(S.start.place_state()))
            if path == "/api/place/picture":           # (e13) the picture of the first click, beside the live one
                from urllib.parse import parse_qs
                q_ = parse_qs(urlparse(self.path).query)
                b = S.start.place_picture((q_.get("robot") or [""])[0], (q_.get("cam") or ["overhead"])[0])
                return self._send(200, b, "image/jpeg") if b else self._send(404, {"error": "no picture"})
            if path == "/api/frame":
                from urllib.parse import parse_qs
                cam = (parse_qs(urlparse(self.path).query).get("cam") or ["overhead"])[0]
                b = S.frame(cam)
                return self._send(200, b, "image/jpeg") if b else self._send(404, {"error": "no picture"})
            if path == "/api/lighting":
                return self._send(200, clean(S.lighting()))
            if path == "/api/mapping":
                return self._send(200, S.mapping())
            if path == "/api/setup":
                return self._send(200, S.setup_state())
            if path == "/api/setup/corner":
                return self._send(200, clean(S.corner.state()))
            if path == "/api/tags/picture":
                from urllib.parse import parse_qs
                cam = (parse_qs(urlparse(self.path).query).get("cam") or ["overhead"])[0]
                b = S.tag.picture_jpeg(cam) if S.tag else None
                return self._send(200, b, "image/jpeg") if b else self._send(404, {"error": "no picture"})
            if path == "/api/tags/chart":
                from urllib.parse import parse_qs
                f = (parse_qs(urlparse(self.path).query).get("f") or [""])[0]
                if S.tag and f and os.path.abspath(f).startswith(os.path.abspath(S.dir)) and os.path.exists(f):
                    with open(f, "rb") as fh:
                        return self._send(200, fh.read(), "image/png")
                return self._send(404, {"error": "no chart"})
            if path == "/api/feel":
                if not S.feel:
                    raise ValueError("this site does not use tags (sites.py: tracking)")
                from urllib.parse import parse_qs
                return self._send(200, clean(S.feel.status((parse_qs(urlparse(self.path).query).get("conn") or
                                                            [None])[0])))
            if path == "/api/sync":
                return self._send(200, clean(S.tag.sync_state()) if S.tag else {})
            if path == "/api/setup/picture":
                from urllib.parse import parse_qs
                cam = (parse_qs(urlparse(self.path).query).get("cam") or ["overhead"])[0]
                b = S.corner.picture(cam)
                return self._send(200, b, "image/jpeg") if b else self._send(404, {"error": "no picture"})
            return self._send(404, {"error": "not found"})
        except Exception as e:  # noqa: BLE001
            return self._send(400, {"error": str(e)})

    def do_POST(self):
        path = urlparse(self.path).path
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost") + tuple(getattr(self, "allow_hosts", ())):
            return self._send(403, {"error": "refused: changes must come from the Mission Control page"})
        n = int(self.headers.get("Content-Length") or 0)
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._send(400, {"error": "bad JSON"})
        S = self.session
        try:
            with S.lock:
                if path == "/api/start/run":
                    return self._send(200, clean(S.start.start(d.get("kind"), attended=bool(d.get("attended")),
                                                               order=d.get("order") or "",
                                                               keep_unreadable=bool(d.get("keep_unreadable")),
                                                               clear_space=d.get("clear_space"))))
                if path == "/api/visit/preset":
                    return self._send(200, clean({"presets": S.start.save_visit_preset(d.get("name"), d.get("order"))}))
                if path == "/api/restart":
                    # (e12) a fresh Mission Control on the new code: only when no run is going
                    if run_going(S):
                        return self._send(409, {"error": "a run is going: Mission Control restarts after it ends, "
                                                         "never mid-run"})
                    restart_self(S)
                    return self._send(200, {"restarting": True, "words": "Mission Control is restarting on the new "
                                                                         "code: the page reloads in a few seconds"})
                if path == "/api/start/stop":
                    return self._send(200, clean(S.start.stop_all()))
                if path == "/api/start/answer":
                    # (e08, A2) a person's answer to a run's question (safety_rules.Questions)
                    import safety_rules as SR_
                    SR_.write_answer(str(d.get("id") or ""), str(d.get("choice") or ""))
                    return self._send(200, clean(S.start.state()))
                if path == "/api/start/prefs":
                    # (e10, C5) the preferences page: saved per site on the Pi
                    S.start.set_prefs(d)
                    return self._send(200, clean(S.start.state()))
                if path == "/api/start/quiz":
                    # (e10, C5) the first-time quiz's answers fill the preferences page in
                    S.start.quiz(d.get("answers") or {})
                    return self._send(200, clean(S.start.state()))
                if path == "/api/start/advanced":
                    S.start.set_advanced(d)
                    import sites as SI_
                    for k_, sk_ in (("measure_trust", "measure_trust"), ("corner_clicks", "corner_clicks"),
                                    ("min_firmware", "calibration.min_firmware")):
                        if k_ in d and S.site:
                            v_ = d[k_]
                            v_ = None if v_ in ("auto", "") else ({"on": True, "off": False}.get(v_, v_)
                                                                   if k_ == "corner_clicks" else
                                                                   ("" if v_ == "none" else v_))
                            SI_.set_override(S.site, sk_, v_)
                            S.site.update(SI_.load(S.site["name"]))
                    return self._send(200, clean(S.start.state()))
                if path == "/api/start/typed":
                    import sites as SI_
                    if not S.site:
                        raise ValueError("no site loaded")
                    key, val = d.get("key"), d.get("value")
                    if key in ("table_length_cm", "table_width_cm"):
                        tb = dict(S.site.get("table") or {})
                        tb["length_cm" if key == "table_length_cm" else "width_cm"] = float(val) if val not in (
                            None, "") else None
                        tb = tb if tb.get("length_cm") and tb.get("width_cm") else None
                        SI_.set_override(S.site, "table", tb)
                    elif key == "tag_height_cm":
                        SI_.set_override(S.site, "tags.height_cm", float(val) if val not in (None, "") else None)
                    else:
                        raise ValueError("Advanced cannot change %s here" % key)
                    S.site.update(SI_.load(S.site["name"]))
                    return self._send(200, clean(S.start.state()))
                if path == "/api/start/site":
                    import site_detect as SD
                    SD.set_pin(d.get("pin") or None)
                    return self._send(200, {"pinned": SD.pinned_site(), "words": "Saved: restart Mission Control "
                                            "(./run-fleet-pi.sh mc-stop, then ./run-fleet-pi.sh mc) to use it."})
                if path == "/api/map":
                    S.set_map(d)
                    return self._send(200, S.state())
                if path == "/api/settings":
                    mark_by_hand(S.settings, d)          # (12 Nov, 1b) upload set by hand is kept as set
                    for k in S.settings:
                        S.settings[k].update((d.get(k) or {}))
                    S.save_settings()
                    S.lock_gaps("saved from the page")      # the 8 cm lock: raised, saved again, said on the page
                    return self._send(200, S.state())
                if path == "/api/warmup/plan":
                    return self._send(200, S.warmup_plan())
                if path == "/api/warmup/analyze":
                    return self._send(200, S.warmup_analyze(d.get("folder")))
                if path == "/api/warmup/apply":
                    S.warmup_apply(d.get("keep_zones"), bool(d.get("move_sides")))
                    return self._send(200, S.state())
                if path == "/api/mission":
                    return self._send(200, S.make_mission(d))
                if path == "/api/program/default":
                    return self._send(200, S.default_program(d.get("robots")))
                if path == "/api/calib/start":
                    return self._send(200, S.calib.start(d))
                if path == "/api/calib/stop":
                    return self._send(200, S.calib.stop())
                if path == "/api/run/start":
                    return self._send(200, S.start_run(d))
                if path == "/api/setup/pictures":
                    return self._send(200, clean(S.corner.pictures(d)))
                if path == "/api/setup/measure":
                    return self._send(200, clean(S.corner.measure(d)))
                if path == "/api/setup/way":
                    S.corner.keep(way="chessboard" if d.get("way") == "chessboard" else "corners")
                    return self._send(200, {"setup_way": S.corner.way})
                if path == "/api/setup/start":
                    return self._send(200, S.setup_start(d))
                if path == "/api/setup/board_away":
                    return self._send(200, S.setup_board_away())
                if path == "/api/place/click":          # (e13) the click (or a known point) for a held robot
                    return self._send(200, clean(S.start.place_click(d, who=self.client_address[0])))
                if path.startswith("/api/tags/") or path.startswith("/api/sync") or path in ("/api/production",
                                                                                         "/api/production/experiment"):
                    if not S.tag:
                        raise ValueError("this site does not use tags (sites.py: tracking)")
                    T = S.tag
                    conn = d.get("conn") or "sim-cameras"
                    if path == "/api/tags/pictures":
                        return self._send(200, clean(T.take_pictures(conn)))
                    if path == "/api/tags/click":
                        return self._send(200, clean(T.set_clicks(d)))
                    if path == "/api/tags/typed":
                        return self._send(200, clean(T.set_typed(d)))
                    if path == "/api/tags/corner_clicks":
                        return self._send(200, clean(T.set_corner_clicks(bool(d.get("on")))))
                    if path == "/api/tags/checkout":
                        return self._send(200, clean(T.checkout(conn)))
                    if path == "/api/tags/corners_confirm":      # (e13) the person confirmed the corners found
                        return self._send(200, clean(T.confirm_corners(d.get("corners") or {})))
                    if path == "/api/tags/measure":
                        r = T.measure(conn)
                        return self._send(200, clean(dict(r, state=S.state())))
                    if path == "/api/tags/refit":
                        folder = d.get("folder") or os.path.dirname(S.last_runs.get("mission") or "") or ""
                        return self._send(200, clean(T.refit(folder, use=bool(d.get("use")))))
                    if path == "/api/tags/room_stop":
                        T.stop_room()
                        return self._send(200, {"room": False})
                    if path == "/api/production":
                        return self._send(200, clean(S.production(d)))
                    if path == "/api/production/experiment":
                        return self._send(200, clean(S.tag.experiment_preview(d)))
                    if path == "/api/sync/mark":
                        return self._send(200, clean(T.mark(d.get("what") or "mark")))
                    if path == "/api/sync/new_id":
                        return self._send(200, clean(T.new_session_id()))
                    if path == "/api/sync/export":
                        import sync_tools as ST
                        folder = d.get("folder") or os.path.dirname(S.last_runs.get("mission") or "")
                        p_, counts = ST.export_timeline(folder)
                        return self._send(200, {"file": p_, "rows": counts})
                    if path == "/api/sync/compare":
                        import merge_external as ME
                        folder = d.get("folder") or os.path.dirname(S.last_runs.get("mission") or "")
                        cdir = os.path.join(folder, "compare")
                        os.makedirs(cdir, exist_ok=True)
                        th = os.path.join(cdir, "external.csv")
                        with open(th, "w") as fh:
                            fh.write(d.get("csv") or "")
                        rf = None
                        if (d.get("refs") or "").strip():
                            rf = os.path.join(cdir, "refs.csv")
                            with open(rf, "w") as fh:
                                fh.write(d["refs"])
                        cfg = ME.default_config()
                        cfg.update(d.get("config") or {})
                        r = ME.run(folder, th, cfg, refs_csv=rf, out=cdir)
                        r["chart"] = os.path.join(cdir, "compare.png")
                        return self._send(200, clean(r))
                if path.startswith("/api/feel/"):
                    if not S.feel:
                        raise ValueError("this site does not use tags (sites.py: tracking)")
                    conn = d.get("conn") or "sim-cameras"
                    if path == "/api/feel/start":
                        if S.run and S.run.alive():
                            raise ValueError("a run is going: stop it first")
                        return self._send(200, clean(S.feel.start(conn, chain=None)))
                    if path == "/api/feel/skip":
                        S.feel.skip_once = bool(d.get("on", True))
                        return self._send(200, clean(S.feel.status(conn)))
                    if path == "/api/feel/again":
                        S.feel.force_next = bool(d.get("on", True))
                        return self._send(200, clean(S.feel.status(conn)))
                if path == "/api/run/pause":
                    return self._send(200, S.pause(bool(d.get("on", True))))
                if path == "/api/run/stop":
                    return self._send(200, S.stop_run() or {})
            return self._send(404, {"error": "not found"})
        except (ValueError, FM.MapError, P.PlanError) as e:
            return self._send(400, {"error": str(e)})
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            return self._send(500, {"error": "%s: %s" % (type(e).__name__, e)})


def detect_where(SD, sim_room=""):
    """(e07) The site from site_detect: a simulated room's evidence, else this computer's cameras and name plus what
    the cameras' pictures show (the recorder's /live, when it answers). A new area gets a site of its own."""
    if sim_room:
        ev = dict(SD.SIM_ROOMS[sim_room])
        ev.setdefault("pinned", SD.pinned_site())
    else:
        ev = SD.hardware_evidence()
        pics = {}
        try:
            import cv2
            import numpy as np
            for cam, port in (("overhead", 8000), ("side", 8001)):
                jpg = grab_jpeg("127.0.0.1", port)
                if jpg is not None:
                    pics[cam] = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
        except Exception:   # noqa: BLE001
            pics = {}
        if pics:
            ev.update(SD.picture_evidence(pics))
    w = SD.decide(ev)
    if w["site"] == "new":
        w["site"] = SD.make_new_site(w.get("template", "school-conference"), ev.get("table_cm"))
        w["words"] += " Its own site: %s." % w["site"]
    w["evidence"] = {k: (sorted(v) if isinstance(v, set) else v) for k, v in ev.items()}
    return w


# ---------------------------------------------------------------------------------------------- (e12) new code after git pull
VERSION = "2026-11-12mc-2"   # (Part 2) the map-first page (mission_control_table.html) at /; the old page at /classic (page_for; Settings > "Open the old page by default", MIRLT_MC_PAGE=classic). Part 3: "2026-11-12mc-3"   # (Part 3) the Pi efficiency monitor beside every run this page starts (monitor_for), /api/pi_health. 1c: "2026-11-12mc-c"   # (1c) /api/live: the live table (live_view.py). 1b: "2026-11-12mc-b"   # (Mission Control build, 12 Nov, 1b) RUN_DEFAULTS upload on; an old unchosen false follows it (upload_default_migrate; settings "by_hand"). 1a: Run.stop: Ctrl+C to the run's whole process group (MIRLT_STOP_GROUP=0: the leader only, as before). Before: "2026-11-12e18"   # e18: the Safety group: one box on the Run screen (More > Safety) with every safety switch, "auto" by default (decided automatically); a switch set there goes to the runs this page starts. Before: "2026-11-10e14"   # e14: the Start button's clear-space answer passed to the run. 2026-11-10e13 (e13): /api/tags/corners_confirm (the corner tags pre-fill, the person confirms); the needs-a-hand box. 2026-11-10a (e12, Nick 10 Nov): notices its own code changed after `git pull` and offers a
                          # restart on the page when no run is going (never mid-run). No version string before
CODE_CHECK_S = 5.0        # chosen: how often the code is looked at (a git call and a few hundred stat calls). No DOE


def code_stamp(repo=None):
    """(git HEAD, newest change time of the code Mission Control and its runs load: tools/**/*.py and the pages)."""
    repo = repo or os.path.dirname(os.path.dirname(HERE))
    head = ""
    try:
        head = subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"], capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    newest = 0.0
    for pat in ("tools/fleet/*.py", "tools/fleet/*.html", "tools/sim/*.py", "tools/pi/*.py", "tools/experiment/*.py",
                "tools/recognition/*.py"):
        for f in glob.glob(os.path.join(repo, pat)):
            try:
                newest = max(newest, os.path.getmtime(f))
            except OSError:
                pass
    return head, newest


class CodeWatch:
    """The code as it was when Mission Control started, against the code now (git pull)."""

    def __init__(self, repo=None):
        self.repo = repo
        self.at_start = code_stamp(repo)
        self.t, self.now = 0.0, self.at_start

    def status(self, running):
        if time.time() - self.t > CODE_CHECK_S:
            self.t, self.now = time.time(), code_stamp(self.repo)
        changed = self.now != self.at_start
        out = {"changed": changed, "running": bool(running), "head_start": self.at_start[0][:8],
               "head_now": self.now[0][:8]}
        if changed:
            how = ("git pull: %s -> %s" % (out["head_start"] or "?", out["head_now"] or "?")
                   if out["head_start"] != out["head_now"] else "files changed")
            out["words"] = ("Mission Control's code changed on the Pi (%s) after it started: the page and new runs "
                            "still use the OLD code until it restarts. %s" % (
                                how,
                                "A run is going: restart AFTER it ends (never mid-run)." if running else
                                "No run is going: press Restart Mission Control now (about 5 seconds)."))
        return out


def run_going(S):
    try:
        return bool(S.start.busy() or (S.run is not None and S.run.alive()) or S.calib.running())
    except Exception:  # noqa: BLE001  (unsure: treat as going, never restart mid-run)
        return True


def restart_self(S, delay_s=0.7):
    """Replace this process with a fresh Mission Control on the same arguments (after the answer has gone)."""
    def go():
        time.sleep(delay_s)
        try:
            srv = getattr(S, "server", None)
            if srv is not None:
                srv.server_close()
        except OSError:
            pass
        sys.stdout.flush()
        os.execv(sys.executable, [sys.executable] + list(sys.argv))
    threading.Thread(target=go, daemon=True).start()


def serve(session, port=PORT, listen="127.0.0.1", allow_hosts=()):
    H = type("H", (Handler,), {"session": session, "allow_hosts": tuple(allow_hosts)})
    srv = ThreadingHTTPServer((listen, port), H)
    return srv, "http://%s:%d/" % (listen if listen != "0.0.0.0" else (list(allow_hosts) or ["127.0.0.1"])[0],
                                   srv.server_address[1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", default="", help="folder for map.json, missions, plans and runs (default: the site's)")
    ap.add_argument("--site", default="", help="the site profile (default: the one in use; tools/fleet/sites.py)")
    ap.add_argument("--map", help="start from this map (copied into --dir)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--listen", default="127.0.0.1", help="address to serve on (the Pi: 0.0.0.0, reached over Tailscale)")
    ap.add_argument("--allow-host", action="append", default=[],
                    help="a Host name besides 127.0.0.1 from which the page may change things (the Pi's Tailscale address)")
    ap.add_argument("--dry", action="store_true", help="open on the simulator even where real cameras answer "
                                                       "(FLEET_DRY=1 does the same)")
    ap.add_argument("--sim-room", choices=("home", "school", "new", "school-misnamed"), default="",
                    help="(e07) practice: the evidence a simulated room shows (site_detect.SIM_ROOMS); implies --dry")
    a = ap.parse_args(argv)
    a.dir_given = bool(a.dir)
    where = None
    if a.sim_room:
        a.dry = True
        a.site = a.site or "auto"
    if a.site == "auto":
        # (e07, Nick 6 Nov) where am I: home, school or a new area, from what the cameras and this computer show
        import site_detect as SD
        where = detect_where(SD, a.sim_room)
        os.environ["MIRLT_SITE"] = where["site"]
        print("WHERE: %s" % where["words"], flush=True)
        if not a.dir_given:
            a.dir = ""                               # the detected site's own folder
    elif a.site:
        os.environ["MIRLT_SITE"] = a.site
    if not a.dir:
        import sites as SI
        a.dir = SI.load()["mc_dir"]
    s = Session(a.dir, a.map, a.port)
    s.force_dry = a.dry
    import start_job as SJB
    s.start = SJB.Starter(s, where=where or {"site": (s.site or {}).get("name", "home"), "kind": "fixed",
                                             "words": "Site chosen at start: %s" % (s.site or {}).get("title", "home")},
                          practice=True if a.dry else None)
    if not a.dry:
        threading.Thread(target=idle_sleeper, args=(s,), daemon=True).start()
    print("Connect screen opens on: %s" % s.conn_default()["why"], flush=True)
    srv, url = serve(s, a.port, a.listen, a.allow_host)
    s.server = srv
    s.code_watch = CodeWatch()                 # (e12) the code as it is now: a git pull later is noticed
    print("Mission Control: %s  (Ctrl+C to stop)" % url, flush=True)

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)       # (killed or closed: the robots stop as with Ctrl+C)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if s.run and s.run.alive():
            s.run.stop()                         # every robot stops (run_plan's own Ctrl+C)
            for _ in range(150):
                if not s.run.alive():
                    break
                time.sleep(0.1)


if __name__ == "__main__":
    main()
