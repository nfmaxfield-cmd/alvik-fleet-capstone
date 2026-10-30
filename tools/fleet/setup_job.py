#!/usr/bin/env python3
"""Mission Control's Setup screen, corners only (Nick, 2026-10-29 20:20-20:27): the user does nothing but click the
table's 4 corners, 1-2-3-4, in the overhead picture and the same 1-4 in the side picture. Everything else is measured
by tools/fleet/corner_setup.py with the robots (their lights, a 10 cm roll each, their bodies): the table's size, both
cameras' poses (height and tilt), the cameras lined up on the robots' lights. Then the warm-up (on by default when the
corners are a real table's edges) feels the sides, and its answers (open edge / barrier per side) go into the map the
mission is planned on.

The recorder (Nick's run sheet clash, 29 Oct): this setup reads the recorder's own pictures (/live on 8000 / 8001), so
it needs the recorder RUNNING, as the warm-up and the run do; it starts the recorder itself when it is not. The old
chessboard way (`setup_way` = "chessboard", kept behind More until checkerboards are printed) opens the cameras itself:
mission_control.py stops the recorder before it and starts it again after.

    Session.corner = CornerSetup(session)      # mission_control.py
    POST /api/setup/pictures {conn}            # one picture from each camera (the simulator's in a dry run)
    GET  /api/setup/picture?cam=overhead       # that picture (JPEG)
    POST /api/setup/measure {conn, corners: {overhead: [[u, v] x 4], side: [...]}, robots: [...], warmup, size}
    GET  /api/setup                            # progress, the result line
"""
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS, os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition")):
    if p not in sys.path:
        sys.path.insert(0, p)

REC_PORTS = {"overhead": 8000, "side": 8001}     # the recorder's /live streams (camrec.py) on the Pi
SIM_PORTS = {"overhead": 8775, "side": 8776}     # the dry setup's simulated cameras (not the runs' 8765 / 8766)
REC_WAIT_S = 45.0       # the recorder's start (both cameras, rec_alive's check) takes 10-20 s on the Pi. No DOE: a wait


def port_answers(port, host="127.0.0.1", timeout=0.3):
    import socket
    try:
        socket.create_connection((host, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def start_recorder(say=print, runner=None):
    """The recorder running (both cameras' /live answering): started with ./run-fleet-pi.sh rec when it is not.
    Returns (ok, words)."""
    if port_answers(REC_PORTS["overhead"]):
        return True, "the recorder is running"
    say("starting the recorder (./run-fleet-pi.sh rec): the setup reads its pictures")
    run = runner or (lambda: subprocess.run(["./run-fleet-pi.sh", "rec"], cwd=REPO, capture_output=True, text=True,
                                            timeout=REC_WAIT_S + 30))
    try:
        r = run()
        out = (getattr(r, "stdout", "") or "")[-300:]
    except Exception as e:     # noqa: BLE001
        return False, "could not start the recorder: %s" % e
    t0 = time.time()
    while time.time() - t0 < REC_WAIT_S:
        if port_answers(REC_PORTS["overhead"]):
            return True, "recorder started"
        time.sleep(0.5)
    return False, "the recorder did not start (%s): Pi window ./run-fleet-pi.sh rec, then try again" % out.strip()


def stop_recorder(say=print, runner=None):
    """For the old chessboard way, which opens the cameras itself. Returns True when it was running (so it is started
    again after)."""
    if not port_answers(REC_PORTS["overhead"]):
        return False
    say("stopping the recorder for the chessboard way (it opens the cameras itself); started again after")
    run = runner or (lambda: subprocess.run(["tmux", "kill-window", "-t", "fleet:rec"], cwd=REPO, capture_output=True,
                                            timeout=20))
    try:
        run()
    except Exception:     # noqa: BLE001
        pass
    t0 = time.time()
    while port_answers(REC_PORTS["overhead"]) and time.time() - t0 < 15:
        time.sleep(0.5)
    time.sleep(2.0)       # the cameras released
    return True


class CornerSetup:
    def __init__(self, session):
        self.S = session
        self.job = {}
        self.pics = {}            # camera -> (jpeg bytes, (w, h))
        self.sim = None           # (process, starts, true overhead path, true side path, ips, table) in a dry setup
        self.lock = threading.Lock()
        self.path = os.path.join(session.dir, "corner_setup.json")   # the last corners clicked, the way chosen
        try:
            with open(self.path) as f:
                self.saved = json.load(f)
        except (OSError, ValueError):
            self.saved = {}

    def keep(self, **kw):
        self.saved.update(kw)
        with open(self.path, "w") as f:
            json.dump(self.saved, f, indent=1)

    @property
    def way(self):
        return self.saved.get("way", "corners")

    # -- the pictures to click on ---------------------------------------------------------------------------
    def table_truth(self, d):
        """The simulator's table (dry setup only): the workspace's size, or the "size" override."""
        f = self.S.map["floor"]
        return (float(f["x_max"] - f["x_min"]), float(f["y_max"] - f["y_min"]))

    def pictures(self, d):
        """One picture from each camera. Real: the recorder's (started first when it is not running). Dry: a simulated
        board (fleet_sim, both cameras, the map's robots on their start spots) that stays up for the measure."""
        import mission_control as MC
        dry = d.get("conn", "sim") != "robots"
        words = []
        if dry:
            self.stop_sim()
            import corner_setup as CS
            table = self.table_truth(d)
            starts = {r["name"]: [float(v) for v in r["start"][:3]] for r in self.S.map["robots"]}
            out = os.path.join(self.S.dir, "setup-sim")
            p, starts, oc, scal, ips = CS.sim_board(table, out, robots=starts,
                                                    ports=(SIM_PORTS["overhead"], SIM_PORTS["side"]))
            self.sim = dict(proc=p, starts=starts, overhead=oc, side=scal, ips=ips, table=table)
            ports = SIM_PORTS
            words.append("the simulator's table (%g x %g cm) and robots" % table)
        else:
            ok, w = start_recorder(say=words.append)
            words.append(w)
            if not ok:
                raise ValueError(w)
            ports = REC_PORTS
        self.pics = {}
        for c, port in ports.items():
            for _ in range(20):
                b = MC.grab_jpeg("127.0.0.1", port)
                if b:
                    break
                time.sleep(0.3)
            if b:
                import cv2
                import numpy as np
                im = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_GRAYSCALE)
                self.pics[c] = (b, (int(im.shape[1]), int(im.shape[0])))
        if "overhead" not in self.pics:
            raise ValueError("no picture from the overhead camera (%s)" % "; ".join(words))
        self.job = dict(state="pictures", words=words, steps=[], running=False, dry=dry)
        return self.state()

    def picture(self, cam):
        return (self.pics.get(cam) or (None,))[0]

    def stop_sim(self):
        if self.sim:
            import signal
            try:
                os.killpg(self.sim["proc"].pid, signal.SIGTERM)
                self.sim["proc"].wait(timeout=10)
            except Exception:     # noqa: BLE001
                pass
            self.sim = None

    # -- the measure ----------------------------------------------------------------------------------------
    def measure(self, d):
        if self.job.get("running"):
            raise ValueError("the setup is already running")
        corners = d.get("corners") or {}
        if len(corners.get("overhead") or []) != 4:
            raise ValueError("click the table's 4 corners in the overhead picture first (1, 2, 3, 4)")
        if "side" in self.pics and corners.get("side") is not None and len(corners["side"]) not in (0, 4):
            raise ValueError("click all 4 corners in the side picture too, or none (overhead alone)")
        dry = d.get("conn", "sim") != "robots"
        if dry and not self.sim:
            raise ValueError("take the pictures first")
        self.job = dict(state="measuring", running=True, steps=[], dry=dry, result=None, error=None, line=None)
        self.keep(corners={c: v for c, v in corners.items() if v}, conn=d.get("conn"))
        threading.Thread(target=self._measure, args=(dict(d), dry), daemon=True).start()
        return self.state()

    def say(self, text):
        self.job.setdefault("steps", []).append([time.strftime("%H:%M:%S"), text])

    def _measure(self, d, dry):
        import corner_setup as CS
        import e02_camera as CAM
        import ladder as LD
        corners = {c: v for c, v in (d.get("corners") or {}).items() if v and len(v) == 4 and c in self.pics}
        out = os.path.join(self.S.dir, "setup-" + time.strftime("%Y%m%d-%H%M%S"))
        bench = None
        try:
            if dry:
                lens = {"overhead": CAM.load_camera(self.sim["overhead"])}
                if self.sim.get("side") and "side" in corners:
                    lens["side"] = CAM.load_camera(self.sim["side"])
                streams = {c: "127.0.0.1:%d" % SIM_PORTS[c] for c in lens}
                ips = self.sim["ips"]
                lights = CS.SIM_LIGHTS
            else:
                import setup_conf as SC
                # each camera's lens model (K, dist) from the calibration it has now; its pose is measured afresh
                lens = {"overhead": CAM.load_camera(SC.overhead_calib(relative=False)[0])}
                side_path = SC.side_calib()[0]
                if "side" in corners and side_path:
                    lens["side"] = CAM.load_camera(side_path)
                elif "side" in corners:
                    self.say("no lens model for the side camera yet (no side calibration saved): overhead alone")
                ok, w = start_recorder(say=self.say)
                if not ok:
                    raise RuntimeError(w)
                streams = {c: "127.0.0.1:%d" % REC_PORTS[c] for c in lens}
                ips = []                      # found on the room's WiFi (the school's addresses are not known)
                lights = None
            corners = {c: corners[c] for c in lens}
            want = list(d.get("robots") or [r["name"] for r in self.S.map["robots"]])
            bench = LD.Bench(ips, os.path.join(out, "link"))
            bench.wait(lambda: all(n in bench.robots() for n in want), 10)
            names = [n for n in want if n in bench.robots()]
            missing = [n for n in want if n not in names]
            if missing:
                self.say("not heard: %s (left out)" % ", ".join(missing))
            if not names:
                raise RuntimeError("no robot answered: are they switched on and on this network?")
            fw = {}
            for n in names:
                bench.send(n, "status")
            bench.wait(lambda: all(n in bench.fw for n in names), 3)
            fw = {n: (bench.fw.get(n) or (None,))[0] for n in names}
            self.say("robots: %s; measuring (lights, then a %.0f cm roll each and back)" % (", ".join(names), CS.ROLL_CM))
            res = CS.run_live(streams, lens, corners, bench, names, out, say=self.say, lights=lights, dry=dry,
                              firmware=fw)
            bench.close()
            bench = None
            self.job["result"] = {k: v for k, v in res.items() if k not in ("points",)}
            self.job["line"] = res.get("line")
            if dry and res.get("ok"):
                res["_spots"] = json.load(open(os.path.join(out, "observations.json")))["spots"]
                self.job["truth"] = CS.truth_report(res, self.sim["table"], {"overhead": self.sim["overhead"],
                                                                             "side": self.sim.get("side")},
                                                    self.sim["starts"])
                res.pop("_spots", None)
            if not res.get("ok"):
                raise RuntimeError(res.get("line") or "not measured")
            self.save(res, out, dry)
            self.apply_size(res, d)
        except Exception as e:     # noqa: BLE001
            self.job["error"] = str(e)
            self.say("stopped: %s" % e)
        finally:
            if bench is not None:
                bench.close()
            if dry:
                self.stop_sim()
        if not self.job.get("error") and d.get("warmup", True):
            self.warmup(d, dry)
        self.job["running"] = False
        self.job["state"] = "done" if not self.job.get("error") else "stopped"

    def save(self, res, out, dry):
        """The new models as the calibrations in use (real), or in the setup's folder (dry). The side file keeps the
        old one's measured lag (lag_s); its old offset is dropped (the new model is lined up on the lights)."""
        self.job["saved"] = {}
        if dry:
            for c in res["cameras"]:
                self.job["saved"][c] = os.path.join(out, "%s_corners_setup.json" % c)
            return
        import setup_conf as SC
        for c, v in res["cameras"].items():
            d = dict(v["model"])
            d["corner_setup"] = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), size_cm=res["size_cm"],
                                     checks={k: [x["length"], x["width"]] for k, x in res["checks"].items()},
                                     cams_agree_cm=res.get("cams_agree_cm"), logs=out)
            if c == "side":
                old = SC.side_calib()[0]
                try:
                    lag = json.load(open(old)).get("lag_s") if old else None
                except (OSError, ValueError):
                    lag = None
                if lag is not None:
                    d["lag_s"] = lag
                path = SC.new_side_calib_path()
                os.makedirs(os.path.dirname(path), exist_ok=True)
                json.dump(d, open(path, "w"), indent=1)
                SC.save_side_pointer(path)
            else:
                path = SC.new_calib_path()
                os.makedirs(os.path.dirname(path), exist_ok=True)
                json.dump(d, open(path, "w"), indent=1)
                SC.save_calib_pointer(path)
            json.dump(d, open(os.path.join(out, os.path.basename(path)), "w"), indent=1)
            self.job["saved"][c] = path
        self.say("saved as the calibrations in use: %s" % ", ".join(self.job["saved"].values()))

    def apply_size(self, res, d):
        """The measured size into the map: always as the surface the cameras see; as the workspace's table too when the
        corners are the table's real edges (the warm-up on), unless the hidden override under More gives one."""
        L, W = res["size_cm"]
        over = str(d.get("size") or "").strip()
        if over:
            a, b = (float(v) for v in over.lower().replace(" ", "").split("x"))
            self.say("size override from More: %g x %g cm (measured %.1f x %.1f)" % (a, b, L, W))
            L, W = a, b
        with self.S.lock:
            self.S.map["surface_cm"] = [round(L, 1), round(W, 1)]
            if d.get("warmup", True):
                f = dict(self.S.map["floor"])
                f.update(x_min=0.0, y_min=0.0, x_max=round(L, 1), y_max=round(W, 1))
                self.S.set_map({"floor": f})
            else:
                self.S.save_map()
        if d.get("warmup", True):
            self.say("the workspace is now the measured table, %.0f x %.0f cm (x along corner 1 -> 2)" % (L, W))
        else:
            self.say("the surface measures %.0f x %.0f cm; the workspace's table (marked on it) is unchanged" % (L, W))

    def warmup(self, d, dry):
        """The warm-up feels the real table's edges; its answers (each side open edge / barrier, the edges found) go
        into the map the mission is planned on. Proposed no-go zones stay proposed (Nick confirms them on the Warm-up
        screen); the sides stay where the corners put them (the edge sensor stops a few cm early)."""
        S = self.S
        self.say("the warm-up: the robots feel the table's sides")
        try:
            with S.lock:
                S.warmup_plan()
                S.start_run({"kind": "warmup", "conn": d.get("run_conn") or ("sim" if dry else "robots"), "dry": dry})
            t0 = time.time()
            while S.run is not None and S.run.alive() and time.time() - t0 < 900:
                time.sleep(1.0)
            with S.lock:
                res = S.warmup_analyze()
                S.warmup_apply(None, move_sides=False)
            for w in res.get("words") or []:
                self.say("   " + w)
            self.say("the warm-up's sides are in the map: the mission plans with them")
            self.job["warmup"] = {"words": res.get("words"), "folder": res.get("folder")}
        except Exception as e:     # noqa: BLE001
            self.job["error"] = "warm-up: %s" % e
            self.say("warm-up not finished: %s" % e)

    def state(self):
        j = dict(self.job)
        j["pictures"] = {c: {"size": v[1]} for c, v in self.pics.items()}
        j["last_corners"] = self.saved.get("corners") or {}
        j["way"] = self.way
        return j
