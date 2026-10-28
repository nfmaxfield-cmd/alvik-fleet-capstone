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
}
WARMUP_KNOBS = ("PROBE_SPACING_CM", "CORNER_PAUSE_S", "BARRIER_NEAR_CM", "SCALE_FLAG", "MAX_FRAMES")


def now_tag():
    return dt.datetime.now().strftime("%Y%m%d-%H%M%S")


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
              "elapsed_s": round(time.time() - self.t0, 1), "exit": self.p.poll()}
        tr = os.path.join(self.folder, "cam_track.csv")
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
        cj = os.path.join(self.folder, "coordinator.jsonl")
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
        sm = os.path.join(self.folder, "run_plan_summary.json")
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
               "table_sizes": TABLE_SIZES,
               "mission": (self.mission or {}).get("mission"), "preview": self.preview,
               "warmup": self.warm_result, "run": self.run.status() if self.run else None,
               "last_runs": self.last_runs}
        return out

    # -- workspace ---------------------------------------------------------
    def set_map(self, d):
        m = copy.deepcopy(self.map)
        m["tasks"] = {}
        for k in ("floor", "sides", "spots", "keepouts", "robots", "open_margin_cm", "notes"):
            if k in d:
                m[k] = d[k]
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
                mm["tasks"] = {k: {"visit": v, "park": True} for k, v in MS.chicken(m, s=s).items()}
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
        plan = P.make_plan(mm, over=over, t_limit_s=float(d.get("t_limit_s", 60)))
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
            lab = (st.get("label") or "").split(" lane")[0].split(" ready")[0].split(" swerve")[0].split(" aside")[0]
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
            "starts": starts, "acts": acts, "summary": sm, "closest_pair": plan["summary"].get("closest_pair"),
            "warnings": plan.get("warnings", []), "planning_time_s": plan["summary"].get("planning_time_s"),
            "notes": (mm.get("mission") or {}).get("notes", []), "figures": (mm.get("mission") or {}).get("figures", []),
            "floor": mm["floor"], "obstacles": mm.get("obstacles", []), "keepouts": mm.get("keepouts", [])}


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
            return self._send(404, {"error": "not found"})
        except Exception as e:  # noqa: BLE001
            return self._send(400, {"error": str(e)})

    def do_POST(self):
        path = urlparse(self.path).path
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost"):
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


def serve(session, port=PORT):
    H = type("H", (Handler,), {"session": session})
    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    return srv, "http://127.0.0.1:%d/" % srv.server_address[1]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", required=True, help="folder for map.json, missions, plans and runs")
    ap.add_argument("--map", help="start from this map (copied into --dir)")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args(argv)
    s = Session(a.dir, a.map, a.port)
    srv, url = serve(s, a.port)
    print("Mission Control: %s  (Ctrl+C to stop)" % url, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if s.run and s.run.alive():
            s.run.stop()


if __name__ == "__main__":
    main()
