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

PAGE = os.path.join(HERE, "mission_control.html")
PORT = 8095
PREVIEW_STEP_S = 0.25     # the preview's sampling of the plan (the slider's resolution). No DOE: display only
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
    "farm": False,                    # robots: each camera tracked in its own process (trackfarm.py)
    "upload": False,                  # robots: upload the run's logs (a release with a tag of its own)
}
WARMUP_KNOBS = ("PROBE_SPACING_CM", "CORNER_PAUSE_S", "BARRIER_NEAR_CM", "SCALE_FLAG", "MAX_FRAMES")


def now_tag():
    return dt.datetime.now().strftime("%Y%m%d-%H%M%S")


START_IN_CM = 12.0     # a start spot carried to a new table size stays this far inside it (a robot's half-width and
                       # room to turn). No DOE: geometry


class Run:
    """run_plan.py in a subprocess (robots or the simulator), its session folder, Pause / Resume / Stop."""

    def __init__(self, kind, folder, cmd):
        self.kind, self.folder, self.cmd = kind, folder, cmd
        os.makedirs(folder, exist_ok=True)
        self.out = open(os.path.join(folder, "run_plan_stdout.txt"), "w")
        self.p = subprocess.Popen(cmd, stdout=self.out, stderr=subprocess.STDOUT, cwd=REPO,
                                  start_new_session=True)
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
                         "warmup": {k: getattr(WU, k) for k in WARMUP_KNOBS}}
        sp = os.path.join(self.dir, "settings.json")
        if os.path.exists(sp):
            try:
                with open(sp) as f:
                    saved = json.load(f)
                for k in self.settings:
                    self.settings[k].update(saved.get(k) or {})
            except ValueError:
                pass
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
        self.last_runs = {}          # kind -> folder

    # -- files -------------------------------------------------------------
    def save_map(self):
        FM.save(self.map, self.map_path)

    def save_settings(self):
        with open(os.path.join(self.dir, "settings.json"), "w") as f:
            json.dump(self.settings, f, indent=1)

    def apply_warmup_knobs(self):
        for k, v in self.settings["warmup"].items():
            if k in WARMUP_KNOBS:
                setattr(WU, k, type(getattr(WU, k))(v))

    # -- state -------------------------------------------------------------
    def state(self):
        m = self.map
        out = {"map": m, "words": WS.words(m), "settings": self.settings,
               "defaults": {"planner": dict(P.DEFAULT_SETTINGS), "mission": dict(MS.SETTINGS), "run": dict(RUN_DEFAULTS),
                            "warmup": {k: getattr(WU, k) for k in WARMUP_KNOBS}},
               "least_open_room_cm": round(MS.least_open_room(m), 2) if m.get("sides") else None,
               "program": self.program or MS.monday_program(), "blocks": list(MS.BLOCKS),
               "table_sizes": TABLE_SIZES, "setup": self.setup_state(), "surface": self.surface(),
               "mission": (self.mission or {}).get("mission"), "preview": self.preview,
               "warmup": self.warm_result, "run": self.run.status() if self.run else None,
               "last_runs": self.last_runs}
        return out

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
        self.warm_map = WU.plan_tasks(self.map)
        plan = P.make_plan(self.warm_map, over=self.settings["planner"] or None)
        with open(os.path.join(self.dir, "warm_map.json"), "w") as f:
            json.dump(self.warm_map, f, indent=1)
        P.save_plan(plan, os.path.join(self.dir, "warm_plan.json"))
        self.preview = preview(self.warm_map, plan, title="Warm-up")
        return {"preview": self.preview}

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
        plan = MS.plan_synced(mm, over=over, t_limit_s=float(d.get("t_limit_s", 60)))
        self.mission, self.plan = mm, plan
        with open(os.path.join(self.dir, "mission.json"), "w") as f:
            json.dump(mm, f, indent=1)
        P.save_plan(plan, os.path.join(self.dir, "plan.json"))
        self.preview = preview(mm, plan, title=(mm.get("mission") or {}).get("name", mode))
        return {"preview": self.preview}

    def default_program(self, robots=None):
        self.program = MS.monday_program(tuple(robots) if robots else ("Left", "Right"))
        return {"program": self.program}

    # -- runs --------------------------------------------------------------
    def start_run(self, d):
        if self.run and self.run.alive():
            raise ValueError("a run is going: stop it first")
        kind = d.get("kind", "mission")
        dry = bool(d.get("dry", True))
        conn = d.get("conn") or ("sim" if dry else "robots")
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
        self.run = Run(kind, folder, cmd)
        self.last_runs[kind] = folder
        return self.run.status()

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
        if kind == "warmup":
            if not self.warm_map:
                self.warmup_plan()
            cmd += ["--mission", os.path.join(self.dir, "warm_map.json"), "--plan", os.path.join(self.dir, "warm_plan.json"),
                    "--border-cm", "0"]
        else:
            if self.program is None:
                self.program = MS.monday_program()
            pp = os.path.join(self.dir, "program.json")
            with open(pp, "w") as f:
                json.dump(self.program, f, indent=1)
            cmd += ["--map", self.map_path, "--program", pp, "--robots", ",".join(self.program.get("robots", []))]
        if conn == "sim-cameras":
            cmd += ["--dry"]
            if rs.get("sim_washout"):
                cmd += ["--washout"]
            if float(rs.get("sim_drop_pct") or 0):
                cmd += ["--drop-pct", str(rs["sim_drop_pct"])]
            if rs.get("sim_dead"):
                cmd += ["--dead", str(rs["sim_dead"])]
        else:
            if rs.get("farm"):
                cmd += ["--farm"]
            if rs.get("upload"):
                cmd += ["--upload"]
        self.run = Run(kind, folder, cmd)
        self.run.session = os.path.join(folder, "run")
        self.run.mode = conn
        self.last_runs[kind] = os.path.join(folder, "run")
        return self.run.status()

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
        if mode == "sim-cameras":
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
                u, v = model.project((p[0], p[1], 4.0 if cam == "overhead" else 1.85))
                if np.isfinite(u) and np.isfinite(v):
                    rr = int(max(8, 6.5 * model.px_per_cm(p[0], p[1], 4.0)))
                    cv2.circle(img, (int(u), int(v)), rr, col.get(n, (0, 200, 255)), 2, cv2.LINE_AA)
                    cv2.putText(img, n, (int(u) + rr + 2, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col.get(n, (0, 200, 255)),
                                2, cv2.LINE_AA)
        ok, out = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.last_frames[cam] = out.tobytes()
        return out.tobytes() if ok else None

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
            if path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                with S.lock:
                    return self._send(200, S.state())
            if path == "/api/run":
                return self._send(200, S.run.status() if S.run else {})
            if path == "/api/review":
                with S.lock:
                    return self._send(200, S.review())
            if path == "/api/frame":
                from urllib.parse import parse_qs
                cam = (parse_qs(urlparse(self.path).query).get("cam") or ["overhead"])[0]
                b = S.frame(cam)
                return self._send(200, b, "image/jpeg") if b else self._send(404, {"error": "no picture"})
            if path == "/api/mapping":
                return self._send(200, S.mapping())
            if path == "/api/setup":
                return self._send(200, S.setup_state())
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
                if path == "/api/map":
                    S.set_map(d)
                    return self._send(200, S.state())
                if path == "/api/settings":
                    for k in S.settings:
                        S.settings[k].update((d.get(k) or {}))
                    S.save_settings()
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
                if path == "/api/run/start":
                    return self._send(200, S.start_run(d))
                if path == "/api/setup/start":
                    return self._send(200, S.setup_start(d))
                if path == "/api/setup/board_away":
                    return self._send(200, S.setup_board_away())
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


def serve(session, port=PORT, listen="127.0.0.1", allow_hosts=()):
    H = type("H", (Handler,), {"session": session, "allow_hosts": tuple(allow_hosts)})
    srv = ThreadingHTTPServer((listen, port), H)
    return srv, "http://%s:%d/" % (listen if listen != "0.0.0.0" else (list(allow_hosts) or ["127.0.0.1"])[0],
                                   srv.server_address[1])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", required=True, help="folder for map.json, missions, plans and runs")
    ap.add_argument("--map", help="start from this map (copied into --dir)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--listen", default="127.0.0.1", help="address to serve on (the Pi: 0.0.0.0, reached over Tailscale)")
    ap.add_argument("--allow-host", action="append", default=[],
                    help="a Host name besides 127.0.0.1 from which the page may change things (the Pi's Tailscale address)")
    a = ap.parse_args(argv)
    s = Session(a.dir, a.map, a.port)
    srv, url = serve(s, a.port, a.listen, a.allow_host)
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
