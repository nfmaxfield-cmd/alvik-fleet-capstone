"""Mission Control's calibration card (2026-11-04): starts tools/fleet/calib_run.py (the cautious calibration run,
zones 1-2) from the Production screen, shows its phase, the bound robot and the plain-words result, and serves its
report and charts. One at a time, and never while a Mission Control run holds the robots' WiFi port.

The simulator run goes in its own network namespace (unshare -rn) when that works, so its UDP ports never meet the
robots' or another run's.

Changelog: 2026-11-12mc (Mission Control build) the Pi efficiency monitor starts beside each calibration run
(pi_monitor.csv in its folder). No version string before."""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

VERSION = "calib-job-2026-11-12mc"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SAFE_FILES = ("report.md", "chart_leg_errors.png", "chart_four_points.png", "chart_tracks.png", "fit.json",
              "binding.json", "status.json", "four_points.json")


def _ns_ok():
    if not shutil.which("unshare"):
        return False
    try:
        return subprocess.run(["unshare", "-rn", "true"], capture_output=True, timeout=5).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


class CalibJob:
    def __init__(self, session):
        self.S = session
        self.p = None
        self.folder = None
        self.out = None
        self.dry = False
        self.t0 = None
        base = os.path.join(session.dir, "calib")
        if os.path.isdir(base):       # the newest earlier one, for its result after a restart
            ds = sorted(d for d in os.listdir(base) if os.path.isdir(os.path.join(base, d)))
            if ds:
                self.folder = os.path.join(base, ds[-1])

    def running(self):
        return self.p is not None and self.p.poll() is None

    def start(self, d):
        if self.running():
            raise ValueError("a calibration run is already running")
        if self.S.run and self.S.run.alive():
            raise ValueError("a Mission Control run is driving the robots: stop it first")
        self.dry = bool(d.get("dry"))
        ts = time.strftime("%Y%m%d-%H%M%S")
        self.folder = os.path.join(self.S.dir, "calib", ts + ("-dry" if self.dry else ""))
        os.makedirs(self.folder, exist_ok=True)
        cmd = [sys.executable, os.path.join(HERE, "calib_run.py"), "--session", self.folder]
        for ip in d.get("robots") or []:
            cmd += ["--robot", str(ip)]
        if self.dry:
            cmd += ["--dry"]
            if d.get("cam_z_cm"):
                cmd += ["--cam-z-cm", str(float(d["cam_z_cm"]))]
            if d.get("quick"):
                cmd += ["--quick"]
            if _ns_ok():
                cmd = ["unshare", "-rn"] + cmd
        else:
            ips = [a for a in (os.environ.get("MIRLT_ROBOT_ARGS") or "").split() if a[:1].isdigit()]
            for ip in ips:
                cmd += ["--robot", ip]
        if d.get("tours"):
            cmd += ["--tours", str(int(d["tours"]))]
        cmd += [str(x) for x in d.get("extra") or []]        # (e07) practice: more simulated robots, breakers
        if d.get("min_firmware") is not None:
            cmd += ["--min-firmware", str(d["min_firmware"])]    # (e07) Advanced: the old override, kept
        self.out = open(os.path.join(self.folder, "calib_stdout.txt"), "w")
        self.p = subprocess.Popen(cmd, stdout=self.out, stderr=subprocess.STDOUT, cwd=REPO, start_new_session=True)
        self.t0 = time.time()
        try:                                       # (12 Nov) the Pi efficiency monitor: pi_monitor.csv in its folder
            import mission_control as MC
            self.monitor = MC.monitor_for(self.folder, self.p.pid)
        except Exception:  # noqa: BLE001
            self.monitor = None
        return self.status()

    def stop(self):
        if self.running():
            os.killpg(self.p.pid, signal.SIGINT)     # its own Ctrl+C: `stop` to the robot, then the report
        return self.status()

    def status(self):
        st = {"running": self.running(), "folder": self.folder, "dry": self.dry}
        if self.folder:
            p = os.path.join(self.folder, "status.json")
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        st.update(json.load(f))
                except ValueError:
                    pass
            st["running"] = self.running()
            st["report"] = os.path.exists(os.path.join(self.folder, "report.md"))
            lp = os.path.join(self.folder, "calib_stdout.txt")
            if os.path.exists(lp):
                with open(lp, errors="replace") as f:
                    st["tail"] = f.read()[-1500:]
        return st

    def file(self, name):
        # (5 Nov) several robots: each robot's own files in its folder, "<robot>/<file>"
        sub, _, base = name.rpartition("/")
        if base not in SAFE_FILES or not self.folder or (sub and not re.fullmatch(r"[A-Za-z0-9_-]{1,20}", sub)):
            raise ValueError("no such file")
        p = os.path.join(self.folder, name)
        if not os.path.exists(p):
            raise ValueError("not written yet")
        with open(p, "rb") as f:
            return f.read()
