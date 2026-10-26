#!/usr/bin/env python3
"""The start page: start and stop robot sessions on the Pi from a phone or a computer (Nick, 2026-10-26: "without
PowerShell or Claude").

An always-on little web server (systemd starts it at boot: tools/pi/install_launcher.sh sets it up once).

Ports (the design):
  * 8091, always: the start page (Start, Stop, status, disk, cameras, batteries). One address to bookmark.
  * 8090 while no session runs: the same start page. A session's live page needs 8090, so this page lets go of it
    just before a session starts (from this page, or from ./run-e02-pi.sh in a terminal, which asks it to:
    `launcher.py --handoff`), and takes it back once the runner has finished. So http://<pi>:8090 always shows
    something useful: the start page when idle, the live page during a session, the upload's progress after.

Starting runs ./run-e02-pi.sh itself (with E02_LAUNCHER=1: no questions in a terminal, no window to attach), so a
session started here is exactly a terminal session: tmux session e02, windows rec and run, the upload at the end.
Stop presses Ctrl+C in the runner's window, as Nick would: every robot stops, everything is saved, then the window
uploads as usual.

Access: only from the home network (private addresses) and Tailscale (100.64.0.0/10), and only with the PIN set at
install (stored on the Pi in ~/.config/mirlt/launcher.json as a salted hash; never in the repository).

    python3 tools/pi/launcher.py                 # serve (systemd does this)
    python3 tools/pi/launcher.py --set-pin       # set or change the PIN
    python3 tools/pi/launcher.py --handoff       # let go of port 8090 now (run-e02-pi.sh calls it)
    python3 tools/pi/launcher.py --addresses     # the page's addresses
"""
import argparse
import glob
import hashlib
import hmac
import ipaddress
import json
import os
import re
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import camnames as CAMS  # noqa: E402

CONF_DIR = os.path.expanduser(os.environ.get("MIRLT_CONF_DIR", "~/.config/mirlt"))
STATE_DIR = os.path.expanduser(os.environ.get("MIRLT_STATE_DIR", "~/.local/state/mirlt"))
PORT = 8091             # chosen: next to the live page's 8090; nothing else on the Pi uses it (camrec 8000/8001/5007,
IDLE_PORT = 8090        # the moment page 8771+)
HANDOFF_GRACE_S = 120   # chosen: after a hand-off, 8090 stays free this long for the runner to open it (the recorder's
                        # 8 s wait plus the runner's start, about 10-20 s on the Pi); a DOE is not needed (any value
                        # well above the start time works)
START_TIMEOUT_S = 180   # run-e02-pi.sh's own work before it hands over to tmux (apt check, git pull, 8 s recorder)
STOP_WAIT_S = 120       # after Stop, the page says "still stopping" past this (the runner's own backstop is 90 s)
PIN_FAILS = 5           # wrong PINs from one address before a 60 s pause (chosen)
COOKIE = "mirlt_launch"
TAILSCALE = ipaddress.ip_network("100.64.0.0/10")
TAILSCALE6 = ipaddress.ip_network("fd7a:115c:a1e0::/48")
ROBOT_CMD_PORT = 5006
ROBOT_LOG_PORT = 5005
ROBOT_NAMES = ("Left", "Center", "Right")
EXTRA_FLAGS = {          # the extra runner options most used (docs, session notes): flag -> plain words
    "--marks": "Tape-mark check before the runs (you put a robot on each tape mark)",
    "--skip-naming": "Skip the naming check (which robot is which)",
    "--skip-find-check": "Skip the find check",
    "--edge-careful": "Edge-careful: never turn on the spot near an open edge",
    "--switches-back": "Back switches on (dock backwards)",
}
SAFE_TOKEN = re.compile(r"^[A-Za-z0-9_.,:=@/+-]{1,80}$")
BLOCKED = {"--dry-run", "--on-pi", "--plan", "--with-side", "--production", "--resume", "--new", "--auto",
           "--view-port"}


# ------------------------------------------------------------------------------------------------ the options
def build_args(opt, unfinished=None):
    """The start page's choices -> (run-e02-pi.sh arguments, extra environment, words). Raises ValueError with a
    plain message for anything not allowed (the arguments end up in a tmux command line: only safe words pass)."""
    args, env, words = [], {}, []
    try:
        hours = float(opt.get("hours", 2))
    except (TypeError, ValueError):
        raise ValueError("Hours must be a number, e.g. 1.5.")
    if not 0.05 <= hours <= 12:
        raise ValueError("Hours must be between 0.05 and 12.")
    hs = ("%g" % hours)
    args += ["--hours", hs]
    words.append("%s hour%s" % (hs, "" if hours == 1 else "s"))
    if opt.get("place", "away") == "home":
        args.append("--home")
        words.append("home (border closer to the edge)")
    else:
        words.append("away")
    prod = bool(opt.get("production"))
    side = bool(opt.get("side"))
    if prod and side:
        raise ValueError("A production session records no video, so the side camera can't be recorded. Pick one.")
    if prod:
        args.append("--production")
        words.append("production (no video)")
    else:
        words.append("with video")
    if side:
        args.append("--with-side")
        words.append("side camera recorded too")
    if opt.get("setup", "explore") == "classic":
        args.append("--classic-setup")
        words.append("classic setup")
    else:
        words.append("explore setup")
    if opt.get("session", "new") == "resume":
        if not unfinished:
            raise ValueError("There is no unfinished session to carry on. Choose 'New session'.")
        args += ["--resume", unfinished["dir"]]
        words.append("carry on %s" % unfinished["name"])
    else:
        args.append("--new")
        words.append("new session")
    robots = opt.get("robots")
    robots = list(ROBOT_NAMES) if robots is None else [r for r in robots if r in ROBOT_NAMES]
    if not robots:
        raise ValueError("Pick at least one robot.")
    if len(robots) < len(ROBOT_NAMES):
        args += ["--robots", ",".join(robots)]
        words.append("robots: " + ", ".join(robots))
    for f in opt.get("extras") or []:
        if f not in EXTRA_FLAGS:
            raise ValueError("Unknown option %s." % f)
        args.append(f)
        words.append(f)
    mr = opt.get("max_runs")
    if mr not in (None, "", 0, "0"):
        try:
            mr = int(mr)
        except (TypeError, ValueError):
            raise ValueError("Max runs must be a whole number.")
        if not 1 <= mr <= 500:
            raise ValueError("Max runs must be between 1 and 500.")
        args += ["--max-runs", str(mr)]
        words.append("at most %d runs" % mr)
    other = (opt.get("other") or "").split()
    for t in other:
        if not SAFE_TOKEN.match(t):
            raise ValueError("'%s' can't go in Other options (letters, numbers and - _ . , : = @ / + only)." % t)
        if t.split("=")[0] in BLOCKED:
            raise ValueError("%s is set by the page's own choices; it can't go in Other options." % t)
    if other:
        if not other[0].startswith("--"):
            raise ValueError("Other options must start with an option name, e.g. --fit-max-rms-cm 2.5.")
        args += other
        words.append(" ".join(other))
    if opt.get("card_ok"):
        env["E02_CARD_OK"] = "y"
        words.append("no USB drive: record to the Pi's card")
    return args, env, words


# ------------------------------------------------------------------------------------------------ what runs now
def scan_procs(proc="/proc"):
    """{'runner': [pid], 'upload': [pid], 'recorder': [pid], 'plan': [pid]} from every process's command line."""
    out = dict(runner=[], upload=[], recorder=[], plan=[])
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return out
    for p in pids:
        try:
            with open(os.path.join(proc, p, "cmdline"), "rb") as f:
                argv = [x.decode("utf-8", "replace") for x in f.read().split(b"\0") if x]
        except OSError:
            continue
        kind = classify(argv)
        if kind:
            out[kind].append(int(p))
    return out


def classify(argv):
    if not argv or not os.path.basename(argv[0]).startswith(("python", "bash", "sh")):
        return None
    for a in argv[1:3]:
        if " " in a:
            continue                       # a shell's command line (bash -c "... e02.py"), not the program
        b = os.path.basename(a)
        if b == "e02.py":
            return "runner"
        if b == "upload_session.py":
            return None if ("--set-token" in argv or "--check-token" in argv) else "upload"
        if b == "camrec.py":
            return None if "--list" in argv else "recorder"
        if b == "session_plan.sh":
            return "plan"
    return None


def tmux(*args, timeout=5):
    try:
        r = subprocess.run(["tmux"] + list(args), capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout
    except (OSError, subprocess.SubprocessError) as e:
        return 1, str(e)


def session_dirs(repo, include_dry=False):
    ds = sorted(d for d in glob.glob(os.path.join(repo, "logs", "e02-*")) if os.path.isdir(d))
    return [d for d in ds if include_dry or "-dryrun" not in os.path.basename(d)]


def read_state(d):
    try:
        with open(os.path.join(d, "e02_state.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def unfinished_session(repo, include_dry=False):
    """The newest session with runs still to go (the runner's own rule: only the newest session is offered)."""
    for d in reversed(session_dirs(repo, include_dry)):
        st = read_state(d)
        if st is None:
            continue
        sch = st.get("schedule") or []
        left = sum(1 for r in sch if r.get("status") in ("pending", "running"))
        if left:
            return dict(dir=d, name=os.path.basename(d), left=left, total=len(sch))
        return None
    return None


def session_summary(d):
    st = read_state(d) or {}
    sch = st.get("schedule") or []
    done = sum(1 for r in sch if r.get("status") == "done")
    left = sum(1 for r in sch if r.get("status") in ("pending", "running"))
    end = st.get("session_end") or {}
    words = end.get("words") or ""
    if not words:
        try:
            with open(os.path.join(d, "e02_log.txt"), errors="replace") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 20000))
                tail = f.read().splitlines()
            for ln in reversed(tail):
                if "Session ended" in ln or "Stopped" in ln or "session done" in ln.lower():
                    words = re.sub(r"^\[[\d:]+\]\s*", "", ln.strip())[:200]
                    if words.rstrip(".").endswith("Ctrl+C"):
                        words = "stopped (the Stop button, or Ctrl+C in its window)"
                    break
        except OSError:
            pass
    return dict(name=os.path.basename(d), runs_done=done, runs_left=left, runs_total=len(sch), end=words,
                holds=len(st.get("holds") or []), mtime=os.path.getmtime(d))


def last_battery(d):
    """Each robot's last battery reading in the session's battery.csv: {robot: (%, time text)}."""
    out = {}
    try:
        with open(os.path.join(d, "battery.csv")) as f:
            head = f.readline().strip().split(",")
            for ln in f:
                row = dict(zip(head, ln.strip().split(",")))
                if row.get("robot") and row.get("battery"):
                    out[row["robot"]] = (row["battery"], row.get("t", ""))
    except OSError:
        pass
    return out


def robot_ips(script):
    try:
        with open(script) as f:
            s = f.read()
    except OSError:
        return []
    m = re.search(r'RUN_ARGS_DEFAULT="([^"]*)"', s)
    return re.findall(r"--robot\s+(\d+\.\d+\.\d+\.\d+)", m.group(1)) if m else []


def ask_batteries(ips, wait_s=4.0):
    """Ask each robot for its status (the firmware's `status`: '... battery 87%, ...'). Only while no session runs:
    the robots answer whichever computer asked last, and the port is the runner's. Returns ({ip or name: %}, words)."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("", ROBOT_LOG_PORT))
    except OSError:
        rx.close()
        return {}, "port %d is in use (a session or the console is running)" % ROBOT_LOG_PORT
    rx.settimeout(0.5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    got = {}
    end = time.time() + wait_s
    last_send = 0.0
    try:
        while time.time() < end and len(got) < len(ips):
            if time.time() - last_send > 1.5:
                for ip in ips:
                    if ip not in got:
                        try:
                            tx.sendto(b"status", (ip, ROBOT_CMD_PORT))
                        except OSError:
                            pass
                last_send = time.time()
            try:
                data, (src, _) = rx.recvfrom(2048)
            except socket.timeout:
                continue
            text = data.decode("utf-8", "replace")
            m = re.search(r"battery (\d+)%", text)
            if m and "status" in text:
                name = text.split("|", 1)[0] if "|" in text else src
                got[src] = dict(name=name, battery=int(m.group(1)))
    finally:
        rx.close()
        tx.close()
    missing = [ip for ip in ips if ip not in got]
    words = "No answer from %s (switched off, asleep or out of WiFi)." % ", ".join(missing) if missing else ""
    return {v["name"]: v["battery"] for v in got.values()}, words


def disk_report(repo, user):
    out = []
    try:
        u = shutil.disk_usage(repo)
        out.append(dict(where="Pi's card", path=repo, free_gb=round(u.free / 1e9, 1), total_gb=round(u.total / 1e9, 1)))
    except OSError:
        pass
    for m in sorted(glob.glob(os.path.join("/media", user, "*"))):
        if os.path.isdir(m):
            try:
                u = shutil.disk_usage(m)
                out.append(dict(where="USB drive " + os.path.basename(m), path=m, free_gb=round(u.free / 1e9, 1),
                                total_gb=round(u.total / 1e9, 1)))
            except OSError:
                pass
    return out


# ------------------------------------------------------------------------------------------------ access
def allowed_ip(ip):
    """Home network (private addresses) and Tailscale only."""
    try:
        a = ipaddress.ip_address(ip.split("%")[0])
    except ValueError:
        return False
    if getattr(a, "ipv4_mapped", None):
        a = a.ipv4_mapped
    if a.is_loopback or a.is_link_local:
        return True
    if a.version == 4:
        return a.is_private or a in TAILSCALE
    return a in TAILSCALE6 or a.is_private


def hash_pin(pin, salt):
    return hashlib.pbkdf2_hmac("sha256", pin.encode(), bytes.fromhex(salt), 200000).hex()


class Conf:
    def __init__(self, path=None):
        self.path = path or os.path.join(CONF_DIR, "launcher.json")
        self.d = {}
        try:
            with open(self.path) as f:
                self.d = json.load(f)
        except (OSError, ValueError):
            self.d = {}

    def has_pin(self):
        return bool(self.d.get("pin_hash"))

    def set_pin(self, pin):
        salt = secrets.token_hex(16)
        self.d.update(pin_salt=salt, pin_hash=hash_pin(pin, salt), secret=secrets.token_hex(32))   # new PIN: old
        os.makedirs(os.path.dirname(self.path), exist_ok=True)                                       # logins end
        tmp = self.path + ".tmp"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump(self.d, f, indent=2)
        os.replace(tmp, self.path)

    def check_pin(self, pin):
        if not self.has_pin():
            return False
        return hmac.compare_digest(hash_pin(pin, self.d["pin_salt"]), self.d["pin_hash"])

    def token(self):
        return hmac.new(bytes.fromhex(self.d.get("secret", "00")), b"mirlt-launch-v1", "sha256").hexdigest()

    def check_token(self, tok):
        return self.has_pin() and bool(tok) and hmac.compare_digest(tok, self.token())


# ------------------------------------------------------------------------------------------------ the launcher
class Launcher:
    def __init__(self, repo=REPO, script=None, conf=None, include_dry=False, user=None, live_port=IDLE_PORT,
                 proc_scan=scan_procs, tmux_fn=tmux, cams_fn=None, ask_fn=ask_batteries, state_dir=None):
        self.repo = repo
        self.script = script or os.path.join(repo, "run-e02-pi.sh")
        self.conf = conf or Conf()
        self.include_dry = include_dry
        self.user = user or os.environ.get("USER") or os.path.basename(os.path.expanduser("~"))
        self.live_port = live_port
        self.proc_scan = proc_scan
        self.tmux = tmux_fn
        self.cams_fn = cams_fn or CAMS.report
        self.ask_fn = ask_fn
        self.state_dir = state_dir or STATE_DIR
        self.lock = threading.Lock()
        self.starting = None          # dict(t, words, log) while run-e02-pi.sh works
        self.last_start = None        # dict(t, ok, words, lines)
        self.stop_t = None
        self.grace_until = 0.0
        self.batteries = None         # dict(t, values, words)
        self.bat_busy = False
        self.idle_srv = None
        self.idle_bind = None         # (host, port) for the 8090 server
        self.handler = None
        self._cache = {}

    # --- what is going on --------------------------------------------------------------------------------------
    def _cached(self, key, ttl, fn):
        now = time.time()
        c = self._cache.get(key)
        if c and now - c[0] < ttl:
            return c[1]
        v = fn()
        self._cache[key] = (now, v)
        return v

    def procs(self):
        return self.proc_scan()

    def upload_progress(self):
        try:
            with open(os.path.join(self.state_dir, "upload.json")) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def phase(self, p=None):
        """idle | starting | running | stopping | uploading | busy (the recorder or a plan without a runner)."""
        p = p or self.procs()
        if self.starting:
            return "starting"
        if p["runner"]:
            if self.stop_t:
                return "stopping"
            return "running"
        if p["upload"]:
            return "uploading"
        if p["plan"] or p["recorder"]:
            return "busy"
        return "idle"

    def live_status(self):
        """The runner's live page (status.json): robots' battery; None if it does not answer."""
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d/status.json" % self.live_port, timeout=1.0) as r:
                return json.loads(r.read().decode())
        except Exception:
            return None

    def status(self):
        p = self.procs()
        ph = self.phase(p)
        if ph not in ("running", "stopping"):
            self.stop_t = None if ph != "uploading" else self.stop_t
        out = dict(phase=ph, t=time.time(), host=socket.gethostname(), live_port=self.live_port)
        up = self.upload_progress()
        out["upload"] = up
        if ph in ("running", "stopping"):
            live = self._cached("live", 2.0, self.live_status)
            out["live_up"] = live is not None
            robots = ((live or {}).get("live") or {}).get("robots") or []
            out["live_robots"] = [dict(name=r.get("name"), battery=r.get("battery"), state=r.get("state"))
                                  for r in robots]
            if live:
                out["live_phase"] = live.get("phase", "")
                q = live.get("question")
                out["question"] = (q or {}).get("prompt", "")[:300] if q else ""
            if self.stop_t:
                out["stop_s"] = round(time.time() - self.stop_t)
                out["stop_slow"] = time.time() - self.stop_t > STOP_WAIT_S
        if p["plan"]:
            out["plan"] = True
        dirs = session_dirs(self.repo, self.include_dry)
        out["last"] = session_summary(dirs[-1]) if dirs else None
        if out["last"] and up and up.get("session") == out["last"]["name"]:
            out["last"]["upload"] = up
        out["unfinished"] = unfinished_session(self.repo, self.include_dry)
        if out["unfinished"]:
            out["unfinished"] = {k: v for k, v in out["unfinished"].items() if k != "dir"}
        out["disk"] = self._cached("disk", 10.0, lambda: disk_report(self.repo, self.user))
        out["cams"] = self._cached("cams", 3.0, self.cams_fn)
        out["batteries"] = self.batteries
        out["bat_last_session"] = last_battery(dirs[-1]) if dirs else {}
        out["bat_busy"] = self.bat_busy
        out["starting"] = dict(words=self.starting["words"], s=round(time.time() - self.starting["t"])) \
            if self.starting else None
        out["last_start"] = self.last_start
        rc, _ = self.tmux("has-session", "-t", "e02")
        out["tmux"] = rc == 0
        out["idle_port_open"] = self.idle_srv is not None
        return out

    # --- start -------------------------------------------------------------------------------------------------
    def check_can_start(self, opt):
        """None if a start is allowed now, else the plain reason."""
        p = self.procs()
        ph = self.phase(p)
        if ph == "starting":
            return "A session is starting already."
        if ph in ("running", "stopping"):
            return "A session is running. Stop it first (or let it finish)."
        if ph == "uploading":
            return "The last session is still uploading. Wait for the upload to finish (its progress is above)."
        if ph == "busy":
            return ("The camera recorder (or a plan) is still running without a runner. Stop it first: the Stop "
                    "button ends it.")
        cams = self.cams_fn()
        if not cams["roles"]["overhead"]["ok"]:
            return "Can't start: " + cams["roles"]["overhead"]["words"] + "."
        if opt.get("side") and not cams["roles"]["side"]["ok"]:
            return "Can't start: " + cams["roles"]["side"]["words"] + " (or choose Side camera: Without)."
        return None

    def start(self, opt):
        """Returns (ok, words). Starts run-e02-pi.sh in the background; the page follows its progress."""
        with self.lock:
            why = self.check_can_start(opt)
            if why:
                return False, why
            try:
                args, env, words = build_args(opt, unfinished_session(self.repo, self.include_dry))
            except ValueError as e:
                return False, str(e)
            rc, _ = self.tmux("has-session", "-t", "e02")
            if rc == 0:
                # the windows of a finished session, waiting for Enter: nothing runs in them (checked above)
                self.tmux("kill-session", "-t", "e02")
            os.makedirs(os.path.join(self.state_dir), exist_ok=True)
            log = os.path.join(self.state_dir, "start-%s.log" % time.strftime("%Y%m%d-%H%M%S"))
            self.starting = dict(t=time.time(), words=", ".join(words), log=log, args=args)
            self.last_start = None
            self.stop_t = None
        self.handoff()
        threading.Thread(target=self._run_start, args=(args, env, log, words), daemon=True).start()
        return True, "Starting: " + ", ".join(words)

    def _run_start(self, args, env, log, words):
        e = dict(os.environ)
        e.update(env)
        e["E02_LAUNCHER"] = "1"
        ok, lines = False, []
        try:
            with open(log, "w") as f:
                f.write("$ %s %s\n" % (self.script, " ".join(args)))
                f.flush()
                r = subprocess.run(["bash", self.script] + args, cwd=self.repo, env=e, stdin=subprocess.DEVNULL,
                                   stdout=f, stderr=subprocess.STDOUT, timeout=START_TIMEOUT_S)
            ok = r.returncode == 0
        except subprocess.TimeoutExpired:
            lines = ["run-e02-pi.sh did not finish starting within %d s." % START_TIMEOUT_S]
        except OSError as ex:
            lines = ["Could not run %s: %s" % (self.script, ex)]
        try:
            with open(log, errors="replace") as f:
                lines = [ln.rstrip() for ln in f.read().splitlines()[1:] if ln.strip()][-14:] + lines
        except OSError:
            pass
        with self.lock:
            self.last_start = dict(t=time.time(), ok=ok, words=", ".join(words), lines=lines,
                                   log=os.path.basename(log))
            self.starting = None
            if not ok:
                self.grace_until = 0.0          # nothing will take 8090: have it back now
        self.tick()

    # --- stop --------------------------------------------------------------------------------------------------
    def stop(self):
        with self.lock:
            p = self.procs()
            ph = self.phase(p)
            if ph == "starting":
                return False, "It is still starting. Wait a few seconds, then Stop."
            if ph == "stopping":
                return False, "Already stopping: every robot is being stopped and saved; the upload follows."
            if ph == "uploading":
                return False, "The session has ended and is uploading. Stopping the upload is not offered here."
            if ph == "running":
                rc, out = self.tmux("send-keys", "-t", "e02:run", "C-c")
                if rc != 0:
                    return False, "Could not reach the runner's window (tmux e02:run): %s" % out.strip()
                self.stop_t = time.time()
                return True, "Stop sent: every robot stops, everything is saved, then it uploads."
            if ph == "busy":
                if p["recorder"]:
                    rc, out = self.tmux("send-keys", "-t", "e02:rec", "C-c")
                    if rc != 0:
                        return False, "Could not reach the recorder's window (tmux e02:rec): %s" % out.strip()
                    return True, "Stop sent to the recorder: it saves the video and closes."
                return False, "A plan is running without a runner right now; stop it in its terminal window."
            return False, "No session is running."

    # --- batteries ---------------------------------------------------------------------------------------------
    def check_batteries(self):
        with self.lock:
            if self.phase() not in ("idle",):
                return False, "Not while a session runs: its live page shows the robots' batteries."
            if self.bat_busy:
                return False, "Already asking."
            self.bat_busy = True

        def go():
            try:
                ips = robot_ips(os.path.join(self.repo, "run-e02-pi.sh")) or robot_ips(self.script)
                vals, words = self.ask_fn(ips) if ips else ({}, "no robot addresses found in run-e02-pi.sh")
                self.batteries = dict(t=time.time(), values=vals, words=words)
            finally:
                self.bat_busy = False
        threading.Thread(target=go, daemon=True).start()
        return True, "Asking the robots (a few seconds) ..."

    # --- port 8090 ---------------------------------------------------------------------------------------------
    def handoff(self):
        """Let go of 8090 so the session's live page can open it."""
        self.grace_until = time.time() + HANDOFF_GRACE_S
        srv, self.idle_srv = self.idle_srv, None
        if srv is not None:
            srv.shutdown()
            srv.server_close()
        return True

    def tick(self):
        """Every 2 s: take 8090 back once nothing needs it; let go if a runner turned up without a hand-off."""
        if self.idle_bind is None:
            return
        p = self.procs()
        if p["runner"]:
            if self.idle_srv is not None:      # a runner started without a hand-off (an older script)
                self.handoff()
            self.grace_until = 0.0             # once it ends, 8090 comes back at once
            return
        if self.idle_srv is None and not self.starting and time.time() >= self.grace_until:
            try:
                srv = ThreadingHTTPServer(self.idle_bind, self.handler)
            except OSError:
                return                     # someone else has it (a live page we did not see): try again later
            srv.daemon_threads = True
            self.idle_srv = srv
            threading.Thread(target=srv.serve_forever, daemon=True).start()

    def watch(self, every_s=2.0):
        while True:
            try:
                self.tick()
            except Exception as e:      # the watcher must never die
                print("launcher tick: %s" % e, flush=True)
            time.sleep(every_s)


# ------------------------------------------------------------------------------------------------ the web side
def make_handler(L, conf, fails=None):
    fails = {} if fails is None else fails

    class H(BaseHTTPRequestHandler):
        server_version = "mirlt-launcher"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json", extra=()):
            if isinstance(body, (dict, list)):
                body = json.dumps(body)
            if isinstance(body, str):
                body = body.encode()
            try:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Length", str(len(body)))
                for k, v in extra:
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _ip(self):
            return self.client_address[0]

        def _authed(self):
            for part in (self.headers.get("Cookie") or "").split(";"):
                k, _, v = part.strip().partition("=")
                if k == COOKIE and conf.check_token(v):
                    return True
            return False

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > 20000:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode() or "{}")
            except ValueError:
                return {}

        def do_GET(self):
            if not allowed_ip(self._ip()):
                return self._send(403, "Only from the home network or Tailscale.", "text/plain")
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(200, PAGE, "text/html; charset=utf-8")
            if path == "/api/status":
                if not self._authed():
                    return self._send(401, dict(error="pin", has_pin=conf.has_pin()))
                return self._send(200, L.status())
            if path == "/api/start-log":
                if not self._authed():
                    return self._send(401, dict(error="pin"))
                ls = L.last_start or {}
                return self._send(200, dict(lines=ls.get("lines", [])))
            return self._send(404, "not here", "text/plain")

        def do_POST(self):
            ip = self._ip()
            path = self.path.split("?")[0]
            if path == "/api/handoff":
                if not ipaddress.ip_address(ip.split("%")[0]).is_loopback:
                    return self._send(403, dict(ok=False))
                L.handoff()
                return self._send(200, dict(ok=True))
            if not allowed_ip(ip):
                return self._send(403, dict(ok=False, words="Only from the home network or Tailscale."))
            if self.headers.get("X-Mirlt") != "1":
                return self._send(403, dict(ok=False, words="Refused (not from the start page)."))
            if path == "/api/login":
                f = fails.get(ip, [0, 0.0])
                if f[0] >= PIN_FAILS and time.time() - f[1] < 60:
                    return self._send(429, dict(ok=False, words="Too many wrong PINs: wait a minute."))
                d = self._body()
                if conf.check_pin(str(d.get("pin") or "")):
                    fails.pop(ip, None)
                    return self._send(200, dict(ok=True), extra=[(
                        "Set-Cookie", "%s=%s; Max-Age=31536000; Path=/; HttpOnly; SameSite=Strict" % (
                            COOKIE, conf.token()))])
                fails[ip] = [f[0] + 1 if time.time() - f[1] < 600 else 1, time.time()]
                words = "Wrong PIN." if conf.has_pin() else "No PIN is set on the Pi yet (install_launcher.sh sets it)."
                return self._send(401, dict(ok=False, words=words))
            if not self._authed():
                return self._send(401, dict(ok=False, words="Enter the PIN first."))
            d = self._body()
            if path == "/api/start":
                if not d.get("confirm"):
                    return self._send(400, dict(ok=False, words="Not confirmed."))
                ok, words = L.start(d.get("options") or {})
                return self._send(200 if ok else 409, dict(ok=ok, words=words))
            if path == "/api/stop":
                if not d.get("confirm"):
                    return self._send(400, dict(ok=False, words="Not confirmed."))
                ok, words = L.stop()
                return self._send(200 if ok else 409, dict(ok=ok, words=words))
            if path == "/api/batteries":
                ok, words = L.check_batteries()
                return self._send(200 if ok else 409, dict(ok=ok, words=words))
            if path == "/api/check":
                # what Start would say, without starting (the confirmation panel shows it)
                why = L.check_can_start(d.get("options") or {})
                try:
                    _, _, words = build_args(d.get("options") or {}, unfinished_session(L.repo, L.include_dry))
                except ValueError as e:
                    why = why or str(e)
                    words = []
                return self._send(200, dict(ok=why is None, words=why or "", summary=words))
            return self._send(404, dict(ok=False))
    return H


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot sessions</title>
<style>
:root{--bg:#f4f5f2;--card:#fff;--ink:#1d2321;--soft:#5d6763;--line:#d9ddd8;--go:#1f7a4d;--go-ink:#fff;
--stop:#b3261e;--warn:#9a5b00;--warn-bg:#fff4dc;--bad-bg:#fde8e6;--ok-bg:#e5f3ea;--accent:#24515f;--sel:#24515f;--sel-ink:#fff}
@media (prefers-color-scheme:dark){:root{--bg:#141817;--card:#1d2321;--ink:#e9ecea;--soft:#a3ada8;--line:#333c38;
--go:#2f9a64;--stop:#d0453a;--warn:#e0a54a;--warn-bg:#3a2e17;--bad-bg:#3d1f1c;--ok-bg:#183326;--accent:#8cc3d1;--sel:#8cc3d1;--sel-ink:#10201f}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:980px;margin:0 auto;padding:16px}
header{display:flex;align-items:baseline;justify-content:space-between;gap:12px;flex-wrap:wrap;margin:4px 0 14px}
h1{font-size:1.45rem;margin:0}
.host{color:var(--soft);font-size:.9rem}
.grid{display:grid;grid-template-columns:1fr;gap:14px}
@media (min-width:860px){.grid{grid-template-columns:3fr 2fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:16px}
.card h2{font-size:1rem;margin:0 0 10px;color:var(--soft);font-weight:600;text-transform:uppercase;letter-spacing:.04em}
.state{display:flex;align-items:center;gap:10px;font-size:1.35rem;font-weight:700}
.dot{width:14px;height:14px;border-radius:50%;background:var(--soft);flex:none}
.idle .dot{background:var(--go)} .running .dot{background:#2a7fd4;animation:p 1.6s infinite}
.starting .dot,.stopping .dot,.uploading .dot{background:var(--warn);animation:p 1.2s infinite}
.busy .dot{background:var(--warn)}
@keyframes p{50%{opacity:.35}}
.sub{color:var(--soft);margin:6px 0 0}
.row{margin:14px 0 0}
.lbl{font-weight:600;margin:0 0 6px;display:block}
.seg{display:flex;flex-wrap:wrap;gap:6px}
.seg button{flex:1 1 auto;min-width:88px;min-height:46px;border:1px solid var(--line);background:var(--card);color:var(--ink);
border-radius:9px;font:inherit;padding:8px 10px;cursor:pointer}
.seg button[aria-pressed=true]{background:var(--sel);color:var(--sel-ink);border-color:var(--sel);font-weight:600}
.seg button:disabled{opacity:.4;cursor:not-allowed}
.hint{color:var(--soft);font-size:.86rem;margin:4px 0 0}
input[type=number],input[type=text],input[type=password]{font:inherit;padding:10px;border:1px solid var(--line);border-radius:9px;
background:var(--card);color:var(--ink);min-height:46px;width:100%}
.hours{display:flex;gap:6px;flex-wrap:wrap;align-items:center}
.hours input{width:110px;flex:none}
details{margin-top:14px;border-top:1px solid var(--line);padding-top:10px}
summary{cursor:pointer;font-weight:600;min-height:40px;display:flex;align-items:center;gap:8px;list-style:none}
summary::-webkit-details-marker{display:none}
summary::before{content:"\25B8";display:inline-block;transition:transform .15s}
details[open] summary::before{transform:rotate(90deg)}
.idle.blocked .dot{background:var(--stop)}
.chk{display:flex;gap:10px;align-items:flex-start;padding:8px 0;cursor:pointer}
.chk input{width:22px;height:22px;flex:none;margin:1px 0 0}
.big{display:block;width:100%;min-height:62px;border:0;border-radius:12px;font:700 1.25rem system-ui,sans-serif;cursor:pointer;margin-top:16px}
.big.go{background:var(--go);color:var(--go-ink)} .big.stop{background:var(--stop);color:#fff}
.big.plain{background:transparent;color:var(--ink);border:1px solid var(--line);font-weight:600;min-height:52px;font-size:1.05rem}
.big:disabled{opacity:.45;cursor:not-allowed}
.confirm{border:2px solid var(--accent);border-radius:12px;padding:14px;margin-top:14px}
.confirm ul{margin:8px 0 0;padding-left:20px}
.two{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.msg{border-radius:10px;padding:10px 12px;margin-top:12px}
.msg.bad{background:var(--bad-bg)} .msg.warn{background:var(--warn-bg)} .msg.ok{background:var(--ok-bg)}
.bar{height:14px;background:var(--line);border-radius:7px;overflow:hidden;margin-top:8px}
.bar i{display:block;height:100%;background:var(--accent);width:0}
a.btn{display:inline-flex;align-items:center;justify-content:center;min-height:48px;padding:0 18px;border-radius:10px;
background:var(--accent);color:var(--sel-ink);text-decoration:none;font-weight:600;margin-top:12px}
table{width:100%;border-collapse:collapse;font-size:.95rem}
td{padding:6px 0;border-bottom:1px solid var(--line);vertical-align:top}
td:last-child{text-align:right}
.miss{color:var(--stop);font-weight:600}
.small{font-size:.85rem;color:var(--soft)}
pre{white-space:pre-wrap;font-size:.8rem;background:var(--bg);padding:8px;border-radius:8px;max-height:220px;overflow:auto}
.hidden{display:none!important}
.pin{max-width:360px;margin:40px auto}
</style></head><body><main>
<header><h1>Robot sessions</h1><span class="host" id="host"></span></header>

<section id="login" class="card pin hidden">
  <h2>Enter the PIN</h2>
  <p class="sub" style="margin-top:0">The PIN set when the start page was installed on the Pi.</p>
  <input id="pin" type="password" inputmode="numeric" autocomplete="current-password" placeholder="PIN">
  <button class="big go" id="pinGo">Open</button>
  <div id="pinMsg"></div>
</section>

<div id="app" class="grid hidden">
 <div>
  <section class="card" id="statusCard">
    <h2>Now</h2>
    <div id="state" class="state idle"><span class="dot"></span><span id="stateWords">...</span></div>
    <p class="sub" id="stateSub"></p>
    <div id="upWrap" class="hidden"><div class="bar"><i id="upBar"></i></div><p class="small" id="upWords"></p></div>
    <a class="btn hidden" id="liveLink" href="#" target="_blank" rel="noopener">Open the live page</a>
    <div id="camAlert"></div>
    <div id="startMsg"></div>
    <div id="failMsg"></div>
    <button class="big stop hidden" id="stopBtn">Stop the session</button>
    <div class="confirm hidden" id="stopConfirm">
      <strong>Stop the session now?</strong>
      <p class="sub">Every robot stops where it is, everything is saved, then the logs and video upload. A later
      session can carry on the runs that are left.</p>
      <div class="two"><button class="big stop" id="stopYes">Yes, stop</button><button class="big plain" id="stopNo">Keep running</button></div>
    </div>
  </section>

  <section class="card hidden" id="startCard" style="margin-top:14px">
    <h2>Start a session</h2>
    <div id="lockMsg" class="msg warn hidden"></div>
    <div id="startBody">
    <div class="row" id="sessRow"><span class="lbl">Session</span>
      <div class="seg" data-k="session"><button data-v="new">New session</button><button data-v="resume" id="resumeBtn">Carry on</button></div>
      <p class="hint" id="sessHint"></p></div>
    <div class="row"><span class="lbl">How long</span>
      <div class="hours seg" data-k="hours"><button data-v="0.5">½ h</button><button data-v="1">1 h</button><button data-v="1.5">1½ h</button><button data-v="2">2 h</button><button data-v="3">3 h</button>
      <label class="small" style="display:flex;align-items:center;gap:6px">or <input id="hoursIn" type="number" min="0.1" max="12" step="0.25" aria-label="Hours"> h</label></div></div>
    <div class="row"><span class="lbl">Where are you?</span>
      <div class="seg" data-k="place"><button data-v="away">Away</button><button data-v="home">Home</button></div>
      <p class="hint">Home: the safety border sits closer to the board's open edges (you can catch a robot).</p></div>
    <div class="row"><span class="lbl">Setup</span>
      <div class="seg" data-k="setup"><button data-v="explore">Explore</button><button data-v="classic">Classic</button></div>
      <p class="hint">Explore: the robots find the board and calibrate the camera themselves. Classic: the stored calibration.</p></div>
    <div class="row"><span class="lbl">Video</span>
      <div class="seg" data-k="production"><button data-v="0">Record video</button><button data-v="1">Production (no video)</button></div>
      <p class="hint">Production keeps a short clip of every stop and a picture every 30 s, not the whole video.</p></div>
    <div class="row"><span class="lbl">Side camera</span>
      <div class="seg" data-k="side"><button data-v="0">Without</button><button data-v="1" id="sideOn">Record it too</button></div></div>
    <div class="row"><span class="lbl">Robots</span>
      <div class="seg" id="robots"><button data-r="Left">Left</button><button data-r="Center">Center</button><button data-r="Right">Right</button></div></div>
    <details id="more"><summary>More options</summary>
      <div id="extras"></div>
      <div class="row"><span class="lbl">Stop after this many runs (empty: all)</span><input id="maxRuns" type="number" min="1" max="500"></div>
      <div class="row"><span class="lbl">Other runner options</span><input id="other" type="text" placeholder="e.g. --fit-max-rms-cm 2.5" autocapitalize="off" autocomplete="off" spellcheck="false"></div>
      <label class="chk"><input type="checkbox" id="cardOk"><span>No USB drive plugged in: record to the Pi's own card</span></label>
    </details>
    <button class="big go" id="startBtn">Start</button>
    <div class="confirm hidden" id="startConfirm">
      <strong>Start this session?</strong>
      <ul id="sumList"></ul>
      <p class="sub">Robots on their parking spots, switched on; the camera aimed as usual.</p>
      <div id="checkMsg"></div>
      <div class="two"><button class="big go" id="startYes">Yes, start</button><button class="big plain" id="startNo">Back</button></div>
    </div>
    </div>
  </section>
 </div>
 <div>
  <section class="card"><h2 id="lastH">Last session</h2><div id="last" class="small">-</div></section>
  <section class="card" style="margin-top:14px"><h2>Cameras</h2><div id="cams"></div></section>
  <section class="card" style="margin-top:14px"><h2>Robots' batteries</h2><div id="bats"></div>
    <button class="big plain" id="batBtn">Ask the robots now</button></section>
  <section class="card" style="margin-top:14px"><h2>Free space</h2><div id="disk"></div></section>
 </div>
</div>
</main>
<script>
const $=id=>document.getElementById(id);
const EXTRAS=__EXTRAS__;
const opt={session:'new',hours:'2',place:'away',setup:'explore',production:'0',side:'0',robots:['Left','Center','Right']};
try{Object.assign(opt,JSON.parse(localStorage.getItem('mirlt-start')||'{}'))}catch(e){}
let S=null, busy=false;
function esc(s){return String(s==null?'':s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function save(){try{localStorage.setItem('mirlt-start',JSON.stringify({hours:opt.hours,place:opt.place,setup:opt.setup,production:opt.production,side:opt.side,robots:opt.robots}))}catch(e){}}
function api(path,body){return fetch(path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json','X-Mirlt':'1'},
  body:body===undefined?undefined:JSON.stringify(body),cache:'no-store',credentials:'same-origin'}).then(async r=>{let j={};try{j=await r.json()}catch(e){};j._code=r.status;return j})}
function paintSeg(){document.querySelectorAll('.seg[data-k]').forEach(g=>{const k=g.dataset.k;g.querySelectorAll('button').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.v===String(opt[k]))))});
  $('hoursIn').value=opt.hours;
  document.querySelectorAll('#robots button').forEach(b=>b.setAttribute('aria-pressed',String(opt.robots.includes(b.dataset.r))));
  $('sideOn').disabled=opt.production==='1'; if(opt.production==='1'&&opt.side==='1'){opt.side='0';paintSeg()}}
document.querySelectorAll('.seg[data-k]').forEach(g=>g.addEventListener('click',e=>{const b=e.target.closest('button');if(!b||b.disabled)return;opt[g.dataset.k]=b.dataset.v;save();paintSeg();hideConfirm()}));
$('hoursIn').addEventListener('input',e=>{opt.hours=e.target.value;save();paintSeg()});
$('robots').addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;const r=b.dataset.r;opt.robots=opt.robots.includes(r)?opt.robots.filter(x=>x!==r):[...opt.robots,r];save();paintSeg();hideConfirm()});
$('extras').innerHTML=Object.entries(EXTRAS).map(([f,w])=>`<label class="chk"><input type="checkbox" data-f="${esc(f)}"><span>${esc(w)} <span class="small">${esc(f)}</span></span></label>`).join('');
function options(){return Object.assign({},opt,{production:opt.production==='1',side:opt.side==='1',
  extras:[...document.querySelectorAll('#extras input:checked')].map(x=>x.dataset.f),max_runs:$('maxRuns').value,other:$('other').value,card_ok:$('cardOk').checked})}
function hideConfirm(){$('startConfirm').classList.add('hidden');$('startBtn').classList.remove('hidden')}
let msgT=0;function msg(el,cls,words){$(el).innerHTML=words?`<div class="msg ${cls}">${esc(words)}</div>`:'';if(el==='startMsg')msgT=Date.now()}
let lastPhase=null;
$('startBtn').onclick=async()=>{msg('startMsg','','');const r=await api('/api/check',{options:options()});
  $('sumList').innerHTML=(r.summary||[]).map(w=>`<li>${esc(w)}</li>`).join('');
  msg('checkMsg',r.ok?'':'bad',r.ok?'':('Can\'t start: '+r.words));$('startYes').disabled=!r.ok;
  $('startConfirm').classList.remove('hidden');$('startBtn').classList.add('hidden');$('startConfirm').scrollIntoView({block:'nearest',behavior:'smooth'})};
$('startNo').onclick=hideConfirm;
$('startYes').onclick=async()=>{$('startYes').disabled=true;const r=await api('/api/start',{confirm:true,options:options()});hideConfirm();
  msg('startMsg',r.ok?'ok':'bad',r.words||'');window.scrollTo({top:0,behavior:'smooth'});tick()};
$('stopBtn').onclick=()=>{$('stopConfirm').classList.remove('hidden');$('stopBtn').classList.add('hidden')};
$('stopNo').onclick=()=>{$('stopConfirm').classList.add('hidden');$('stopBtn').classList.remove('hidden')};
$('stopYes').onclick=async()=>{$('stopYes').disabled=true;const r=await api('/api/stop',{confirm:true});$('stopConfirm').classList.add('hidden');
  msg('startMsg',r.ok?'ok':'bad',r.words||'');$('stopYes').disabled=false;tick()};
$('batBtn').onclick=async()=>{const r=await api('/api/batteries',{});msg('startMsg',r.ok?'ok':'warn',r.words);setTimeout(tick,500)};
$('pinGo').onclick=async()=>{const r=await api('/api/login',{pin:$('pin').value});if(r.ok){$('pin').value='';tick()}else msg('pinMsg','bad',r.words||'Wrong PIN.')};
$('pin').addEventListener('keydown',e=>{if(e.key==='Enter')$('pinGo').click()});
function ago(t){const s=Math.round(Date.now()/1000-t);if(s<90)return s+' s ago';if(s<5400)return Math.round(s/60)+' min ago';if(s<172800)return Math.round(s/3600)+' h ago';return Math.round(s/86400)+' days ago'}
function gb(x){return x==null?'?':(x>=100?Math.round(x):x)+' GB'}
const WORDS={idle:'Idle: ready to start',starting:'Starting ...',running:'Running',stopping:'Stopping ...',uploading:'Uploading',busy:'Busy'};
function render(s){S=s;$('host').textContent=s.host;
  if(lastPhase!==null&&lastPhase!==s.phase&&Date.now()-msgT>1500)msg('startMsg','','');
  if(msgT&&Date.now()-msgT>20000){msg('startMsg','','');msgT=0}
  lastPhase=s.phase;
  const ro=s.cams.roles;const blocked=s.phase==='idle'&&!ro.overhead.ok;
  const st=$('state');st.className='state '+s.phase+(blocked?' blocked':'');
  $('stateWords').textContent=blocked?'Idle: can\'t start (overhead camera missing)':(WORDS[s.phase]||s.phase);
  $('startBtn').disabled=blocked;$('startBtn').textContent=blocked?'Start (plug in the overhead camera first)':'Start';
  let sub='';
  if(s.phase==='starting'&&s.starting)sub=s.starting.words+' ('+s.starting.s+' s so far: checks, then the recorder, then the runner).';
  if(s.phase==='running')sub=(s.live_up?'The live page is up: watch it and answer its questions there.':'The runner is starting its live page ...')+(s.live_phase?' Now: '+s.live_phase+'.':'')+(s.question?' It is asking: "'+s.question+'"':'');
  if(s.phase==='stopping')sub=s.stop_slow?('Still stopping after '+s.stop_s+' s. The runner ends by itself within about 90 s more; if not, look at its window on the Pi (tmux attach -t e02).'):('Every robot is being stopped and the session saved ('+s.stop_s+' s). The upload starts next.');
  if(s.phase==='busy')sub=s.plan?'A plan of sessions is running (started from a terminal).':'The camera recorder is running without a runner. Stop ends it.';
  if(s.phase==='idle')sub=s.tmux?'The last session\'s windows are still open on the Pi; Start closes them.':'Nothing is running.';
  $('stateSub').textContent=sub;
  const live=(s.phase==='running'||s.phase==='stopping');
  $('liveLink').classList.toggle('hidden',!live);$('liveLink').href=location.protocol+'//'+location.hostname+':'+s.live_port+'/';
  $('stopBtn').classList.toggle('hidden',!(s.phase==='running'||s.phase==='busy')||!$('stopConfirm').classList.contains('hidden'));
  if(!(s.phase==='running'||s.phase==='busy'))$('stopConfirm').classList.add('hidden');
  $('stopBtn').textContent=s.phase==='busy'?'Stop the recorder':'Stop the session';
  const u=s.upload;const upNow=s.phase==='uploading'&&u&&u.state==='uploading';
  $('upWrap').classList.toggle('hidden',!upNow);
  if(upNow){const pc=u.total_bytes?Math.min(99,Math.round(100*u.done_bytes/u.total_bytes)):0;$('upBar').style.width=pc+'%';
    $('upWords').textContent=pc+'%: '+Math.round(u.done_bytes/1e6)+' MB of about '+(u.total_bytes/1e9).toFixed(2)+' GB'+(u.t_start&&u.done_bytes>2e7?(', about '+Math.max(1,Math.round((u.total_bytes-u.done_bytes)/(u.done_bytes/(Date.now()/1000-u.t_start))/60))+' min to go'):'')+(u.current?' (now: '+u.current+')':'')+(u.last_error?' - retrying after: '+u.last_error:'');}
  else if(s.phase==='uploading'){$('upWrap').classList.remove('hidden');$('upBar').style.width='2%';$('upWords').textContent='Packing the logs ...'}
  const LOCK={starting:'A session is starting.',running:'A session is running: a second one can\'t start until it ends.',
    stopping:'The session is stopping; it uploads next.',uploading:'The last session is uploading: a new one can start when the upload is done.',
    busy:'The recorder or a plan is still running.'};
  $('startCard').classList.remove('hidden');const locked=s.phase!=='idle';
  $('startBody').classList.toggle('hidden',locked&&$('startConfirm').classList.contains('hidden'));
  $('lockMsg').classList.toggle('hidden',!locked);$('lockMsg').textContent=locked?('Start is locked. '+(LOCK[s.phase]||'')):'';
  // the session choice
  const uf=s.unfinished;$('resumeBtn').disabled=!uf;if(!uf&&opt.session==='resume'){opt.session='new'}
  $('resumeBtn').textContent=uf?('Carry on '+uf.name.replace('e02-','')):'Carry on (none unfinished)';
  $('sessHint').textContent=uf?(uf.name+' has '+uf.left+' of '+uf.total+' runs left. Carry on runs them; New starts again.'):'No unfinished session.';
  paintSeg();
  // camera alert
  let ca=[];if(!ro.overhead.ok)ca.push(ro.overhead.words.charAt(0).toUpperCase()+ro.overhead.words.slice(1)+'.');
  if(!ro.side.ok&&opt.side==='1'&&s.phase==='idle')ca.push('Side camera missing ("Record it too" is chosen): '+ro.side.words);
  $('camAlert').innerHTML=ca.map((w,i)=>`<div class="msg ${i===0&&!ro.overhead.ok?'bad':'warn'}">${esc(w)}</div>`).join('');
  $('failMsg').innerHTML=(s.last_start&&!s.last_start.ok&&s.phase==='idle')?`<div class="msg bad"><strong>The last start did not go through.</strong> What the Pi said:<pre>${esc((s.last_start.lines||[]).join('\n'))}</pre></div>`:'';
  // last session
  const cur=live||s.phase==='starting';$('lastH').textContent=cur?'This session':'Last session';
  const L=s.last;if(L){const up=L.upload;let w=`<table><tr><td>Session</td><td>${esc(L.name)}</td></tr><tr><td>Runs</td><td>${L.runs_total?L.runs_done+' done of '+L.runs_total+(L.runs_left?', '+L.runs_left+' left':''):'none planned yet (setup)'}</td></tr>`+
    `<tr><td>Holds for you</td><td>${L.holds}</td></tr>`+(cur?'</table>':`<tr><td>Ended</td><td>${esc(L.end||'(not recorded)')}</td></tr><tr><td>Upload</td><td>`+
    (up?(up.state==='done'?`<a href="${esc(up.link)}" target="_blank" rel="noopener">done: open it</a>`:up.state==='uploading'?'uploading ...':`<span class="miss">${esc(up.error||'failed')}</span>`):'no upload recorded')+`</td></tr></table>`);
    $('last').innerHTML=w}else $('last').textContent=cur?'Its folder appears once the runner starts.':'No session yet.';
  // cameras
  $('cams').innerHTML='<table>'+(s.cams.cameras.map(c=>`<tr><td>${esc(c.name)}<div class="small">/dev/video${c.dev}${c.port?' · USB port '+esc(c.port):''}</div></td><td>${c.role?'<strong>'+c.role+'</strong>':'<span class="small">not used</span>'}</td></tr>`).join('')||'<tr><td class="miss">No cameras found</td><td></td></tr>')+'</table>'+
    ['overhead','side'].filter(r=>!ro[r].ok).map(r=>`<p class="miss">${r[0].toUpperCase()+r.slice(1)} camera ("${esc(ro[r].name)}") not found</p>`).join('')+
    '<p class="small">Cameras are picked by name (~/.config/mirlt/cameras.json on the Pi), not by number.</p>';
  // batteries
  let b='';
  if(live&&s.live_robots&&s.live_robots.length){b='<table>'+s.live_robots.map(r=>`<tr><td>${esc(r.name)}<div class="small">${esc(r.state||'')}</div></td><td>${r.battery==null?'?':r.battery+'%'}</td></tr>`).join('')+'</table><p class="small">From the running session.</p>'}
  else if(s.batteries){const v=s.batteries.values||{};b='<table>'+Object.keys(v).sort().map(k=>`<tr><td>${esc(k)}</td><td>${v[k]}%</td></tr>`).join('')+'</table><p class="small">Asked '+ago(s.batteries.t)+'. '+esc(s.batteries.words||'')+'</p>'}
  else{const v=s.bat_last_session||{};const ks=Object.keys(v).sort();b=ks.length?('<table>'+ks.map(k=>`<tr><td>${esc(k)}</td><td>${esc(v[k][0])}%</td></tr>`).join('')+'</table><p class="small">At the end of the last session (old).</p>'):'<p class="small">Not asked yet.</p>'}
  $('bats').innerHTML=b;$('batBtn').classList.toggle('hidden',s.phase!=='idle');$('batBtn').disabled=s.bat_busy;$('batBtn').textContent=s.bat_busy?'Asking ...':'Ask the robots now';
  $('disk').innerHTML='<table>'+s.disk.map(d=>`<tr><td>${esc(d.where)}</td><td><strong>${gb(d.free_gb)}</strong> free of ${gb(d.total_gb)}</td></tr>`).join('')+'</table><p class="small">A 2-hour session with video needs about 7 GB on the USB drive.</p>';
}
async function tick(){const r=await api('/api/status').catch(()=>null);
  if(!r){$('stateWords').textContent='The Pi does not answer';return}
  if(r._code===401){$('login').classList.remove('hidden');$('app').classList.add('hidden');if(!r.has_pin)msg('pinMsg','warn','No PIN set on the Pi yet: run the install (docs/pi-run.md).');return}
  $('login').classList.add('hidden');$('app').classList.remove('hidden');render(r)}
paintSeg();tick();setInterval(()=>{if(!document.hidden)tick()},2000);
</script></body></html>
""".replace("__EXTRAS__", json.dumps(EXTRA_FLAGS))


# ------------------------------------------------------------------------------------------------ command line
def addresses(port=PORT):
    out = []
    try:
        r = subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=5)
        for ln in r.stdout.split():
            out.append(("Tailscale (anywhere)", "http://%s:%d" % (ln.strip(), port)))
    except (OSError, subprocess.SubprocessError):
        pass
    out.append(("Home WiFi", "http://%s.local:%d" % (socket.gethostname(), port)))
    try:
        r = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5)
        for ip in r.stdout.split():
            if ip.count(".") == 3 and not ip.startswith("100.") and not ip.startswith("172.17."):
                out.append(("Home WiFi (by number)", "http://%s:%d" % (ip, port)))
    except (OSError, subprocess.SubprocessError):
        pass
    return out


def set_pin_interactive(conf):
    import getpass
    for _ in range(3):
        p1 = getpass.getpass("Choose a PIN for the start page (4 to 12 digits; it will not show) and press Enter: ")
        p1 = p1.strip()
        if not re.fullmatch(r"\d{4,12}", p1):
            print("It must be 4 to 12 digits (numbers only). Try again.")
            continue
        p2 = getpass.getpass("Type the same PIN again and press Enter: ").strip()
        if p1 != p2:
            print("The two PINs were not the same. Try again.")
            continue
        conf.set_pin(p1)
        print("PIN saved on the Pi (%s). Phones and computers that used the old PIN must enter the new one." % conf.path)
        return 0
    print("PIN not set.")
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--idle-port", type=int, default=IDLE_PORT, help="the live page's port, served while idle (0: off)")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--repo", default=REPO)
    ap.add_argument("--script", default=None, help="the session script (default: run-e02-pi.sh; tests: a dry one)")
    ap.add_argument("--include-dry", action="store_true", help="tests: dry-run sessions count as the last session")
    ap.add_argument("--set-pin", action="store_true")
    ap.add_argument("--handoff", action="store_true", help="ask the running start page to let go of the live port")
    ap.add_argument("--addresses", action="store_true")
    a = ap.parse_args(argv)
    if a.set_pin:
        return set_pin_interactive(Conf())
    if a.addresses:
        for w, u in addresses(a.port):
            print("  %-24s %s" % (w + ":", u))
        return 0
    if a.handoff:
        req = urllib.request.Request("http://127.0.0.1:%d/api/handoff" % a.port, data=b"{}", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                r.read()
            return 0
        except Exception:
            return 1                       # not installed or not running: nothing holds the port
    conf = Conf()
    L = Launcher(repo=a.repo, script=a.script, conf=conf, include_dry=a.include_dry, live_port=a.idle_port or IDLE_PORT)
    H = make_handler(L, conf)
    L.handler = H
    srv = ThreadingHTTPServer((a.bind, a.port), H)
    srv.daemon_threads = True
    if a.idle_port:
        L.idle_bind = (a.bind, a.idle_port)
        L.tick()
    threading.Thread(target=L.watch, daemon=True).start()
    if not conf.has_pin():
        print("No PIN set yet: python3 tools/pi/launcher.py --set-pin", flush=True)
    print("Start page on port %d (and %d while no session runs)." % (a.port, a.idle_port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
