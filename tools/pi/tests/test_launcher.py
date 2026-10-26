"""The start page (tools/pi/launcher.py), cameras by name (tools/pi/camnames.py) and run-e02-pi.sh's camera choice
(2026-10-26)."""
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PI))
sys.path.insert(0, PI)
import camnames as C  # noqa: E402
import launcher as LA  # noqa: E402


# ---------------------------------------------------------------- stand-in cameras (the Pi's 2026-10-26 layout)
def fake_sysfs(root, cams):
    """cams: [(video number, name, index, usb port)] -> a /sys/class/video4linux stand-in."""
    for n, name, idx, port in cams:
        d = os.path.join(root, "video%d" % n)
        os.makedirs(d)
        with open(os.path.join(d, "name"), "w") as f:
            f.write(name + "\n")
        with open(os.path.join(d, "index"), "w") as f:
            f.write("%d\n" % idx)
        iface = os.path.join(root, "_usb", "1-%s:1.0" % port)
        os.makedirs(iface, exist_ok=True)
        os.symlink(iface, os.path.join(d, "device"))
    return root


PI_1026 = [(0, "EMEET SmartCam C950", 0, "1.3"), (1, "EMEET SmartCam C950", 1, "1.3"),
           (2, "USB 2.0 Camera: USB Camera", 0, "1.4"), (3, "USB 2.0 Camera: USB Camera", 1, "1.4")]
SWAPPED = [(0, "USB 2.0 Camera: USB Camera", 0, "1.4"), (1, "USB 2.0 Camera: USB Camera", 1, "1.4"),
           (2, "EMEET SmartCam C950", 0, "1.3"), (3, "EMEET SmartCam C950", 1, "1.3")]


def test_cameras_found_by_name_after_the_numbers_swap(tmp_path):
    conf, _ = C.load_conf(str(tmp_path / "none.json"))
    a = C.scan(fake_sysfs(str(tmp_path / "a"), PI_1026))
    b = C.scan(fake_sysfs(str(tmp_path / "b"), SWAPPED))
    assert [c["dev"] for c in a] == [0, 2]                       # one per camera: its picture node
    assert C.resolve("overhead", a, conf)[0] == 2 and C.resolve("side", a, conf)[0] == 0
    assert C.resolve("overhead", b, conf)[0] == 0 and C.resolve("side", b, conf)[0] == 2
    assert {c["port"] for c in a} == {"1.3", "1.4"}


def test_missing_camera_is_said_plainly(tmp_path):
    cams = C.scan(fake_sysfs(str(tmp_path / "s"), PI_1026[:2]))           # the overhead unplugged
    conf, _ = C.load_conf(str(tmp_path / "none.json"))
    dev, words = C.resolve("overhead", cams, conf)
    assert dev is None and "not plugged in" in words and "EMEET SmartCam C950 at /dev/video0" in words
    rep = C.report(cams, conf)
    assert not rep["roles"]["overhead"]["ok"] and rep["roles"]["side"]["ok"]
    assert rep["cameras"][0]["role"] == "side"


def test_two_cameras_with_the_same_name_need_a_port(tmp_path):
    cams = C.scan(fake_sysfs(str(tmp_path / "s"), [(0, "USB 2.0 Camera", 0, "1.3"), (2, "USB 2.0 Camera", 0, "1.4")]))
    conf = dict(overhead=dict(name="USB 2.0 Camera", port=""), side=dict(name="EMEET", port=""))
    dev, words = C.resolve("overhead", cams, conf)
    assert dev is None and "two cameras" in words
    conf["overhead"]["port"] = "1.4"
    assert C.resolve("overhead", cams, conf)[0] == 2


def test_conf_file_round_trip_and_cli(tmp_path, monkeypatch, capsys):
    path = str(tmp_path / "cameras.json")
    monkeypatch.setattr(C, "CONF_PATH", path)
    monkeypatch.setattr(C, "SYSFS", fake_sysfs(str(tmp_path / "s"), SWAPPED))
    assert C.main(["--init"]) == 0
    assert json.load(open(path)) == {"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}
    capsys.readouterr()
    assert C.main(["--dev", "overhead"]) == 0 and capsys.readouterr().out.strip() == "0"
    assert C.main(["--set", "overhead", "EMEET"]) == 0
    capsys.readouterr()
    assert C.main(["--dev", "overhead"]) == 1          # both roles now name the same camera: refused
    assert "same name" in capsys.readouterr().err


# ---------------------------------------------------------------- run-e02-pi.sh picks the camera by name
def _run_script(tmp_path, cams, env_extra=None, args=("--production",)):
    """run-e02-pi.sh with tmux, git, sudo and sleep stubbed: returns (exit code, output, tmux calls)."""
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    calls = tmp_path / "tmux.calls"
    (bin_ / "tmux").write_text('#!/bin/sh\necho "$@" >> %s\n[ "$1" = has-session ] && exit 1\nexit 0\n' % calls)
    (bin_ / "git").write_text("#!/bin/sh\nexit 1\n")
    (bin_ / "sleep").write_text("#!/bin/sh\nexit 0\n")
    # free space: plenty (the test must not depend on this machine's disk)
    (bin_ / "df").write_text("#!/bin/sh\necho Avail\necho 500G\n")
    for f in ("tmux", "git", "sleep", "df"):
        os.chmod(bin_ / f, 0o755)
    env = dict(os.environ, PATH="%s:%s" % (bin_, os.environ["PATH"]), HOME=str(tmp_path / "home"),
               MIRLT_V4L_SYSFS=fake_sysfs(str(tmp_path / "sys"), cams), MIRLT_CONF_DIR=str(tmp_path / "conf"),
               E02_LAUNCHER="1")
    env.pop("OVERHEAD_DEV", None)
    env.pop("SIDE_DEV", None)
    env.update(env_extra or {})
    r = subprocess.run(["bash", os.path.join(REPO, "run-e02-pi.sh")] + list(args), env=env, capture_output=True,
                       text=True, timeout=120, stdin=subprocess.DEVNULL)
    return r.returncode, r.stdout + r.stderr, (calls.read_text() if calls.exists() else "")


def _deps_ok():
    try:
        import cv2, numpy, scipy, matplotlib  # noqa: F401,E401
        return shutil.which("bash") is not None
    except ImportError:
        return False


@pytest.mark.skipif(not _deps_ok(), reason="needs OpenCV, SciPy, Matplotlib (else the script installs them)")
def test_script_production_uses_the_overhead_by_name(tmp_path):
    rc, out, calls = _run_script(tmp_path, SWAPPED)
    assert rc == 0, out
    assert "Overhead camera: /dev/video0 (found by its name)" in out
    assert "--camera-device 0" in calls and "--production" in calls
    assert "attach" not in calls                                   # E02_LAUNCHER: no window to attach to


@pytest.mark.skipif(not _deps_ok(), reason="needs OpenCV, SciPy, Matplotlib")
def test_script_overhead_dev_still_overrides(tmp_path):
    rc, out, calls = _run_script(tmp_path, PI_1026, dict(OVERHEAD_DEV="5"))
    assert rc == 0, out
    assert "/dev/video5 (OVERHEAD_DEV, by hand)" in out and "--camera-device 5" in calls


@pytest.mark.skipif(not _deps_ok(), reason="needs OpenCV, SciPy, Matplotlib")
def test_script_stops_when_the_overhead_is_missing(tmp_path):
    rc, out, calls = _run_script(tmp_path, PI_1026[:2])
    assert rc == 1
    assert "overhead camera (USB 2.0 Camera) is not plugged in" in out and "OVERHEAD_DEV=2" in out
    assert "new-session" not in calls


# ---------------------------------------------------------------- the options
def test_build_args_defaults_and_choices():
    args, env, words = LA.build_args({})
    assert args == ["--hours", "2", "--new"] and env == {}
    uf = dict(dir="/x/logs/e02-20261025-211400", name="e02-20261025-211400")
    args, env, words = LA.build_args(dict(hours="1.5", place="home", setup="classic", side=True, session="resume",
                                          robots=["Left", "Right"], extras=["--marks"], max_runs="3",
                                          other="--fit-max-rms-cm 2.5", card_ok=True), uf)
    assert args == ["--hours", "1.5", "--home", "--with-side", "--classic-setup", "--resume", uf["dir"],
                    "--robots", "Left,Right", "--marks", "--max-runs", "3", "--fit-max-rms-cm", "2.5"]
    assert env == {"E02_CARD_OK": "y"}
    assert LA.build_args(dict(production=True))[0] == ["--hours", "2", "--production", "--new"]


@pytest.mark.parametrize("opt,words", [
    (dict(hours="x"), "number"), (dict(hours=40), "between"), (dict(production=True, side=True), "side camera"),
    (dict(session="resume"), "no unfinished"), (dict(robots=[]), "at least one"), (dict(extras=["--rm"]), "Unknown"),
    (dict(other="; rm -rf ~"), "can't go"), (dict(other="--dry-run"), "set by the page"),
    (dict(other="--resume=x"), "set by the page"), (dict(other="foo"), "must start"), (dict(max_runs="0.5"), "whole"),
])
def test_build_args_refuses(opt, words):
    with pytest.raises(ValueError) as e:
        LA.build_args(opt)
    assert words in str(e.value)


def test_classify_processes():
    assert LA.classify(["python3", "tools/experiment/e02.py", "--on-pi"]) == "runner"
    assert LA.classify(["/usr/bin/python3", "/home/u/r/tools/pi/upload_session.py", "--recordings", "x"]) == "upload"
    assert LA.classify(["python3", "tools/pi/upload_session.py", "--set-token"]) is None
    assert LA.classify(["python3", "tools/pi/camrec.py", "--list"]) is None
    assert LA.classify(["python3", "tools/pi/camrec.py", "--device", "2"]) == "recorder"
    assert LA.classify(["bash", "tools/pi/session_plan.sh", "/media"]) == "plan"
    assert LA.classify(["vim", "tools/experiment/e02.py"]) is None
    assert LA.classify(["bash", "-c", "cd x && python3 tools/experiment/e02.py"]) is None   # its shell is not it


def test_allowed_ip():
    for ip in ("127.0.0.1", "192.168.1.40", "10.0.0.5", "172.20.1.1", "100.122.222.75", "fd7a:115c:a1e0::5",
               "::ffff:192.168.1.9", "::1"):
        assert LA.allowed_ip(ip), ip
    for ip in ("8.8.8.8", "1.1.1.1", "100.128.0.1", "2001:4860::1", "junk"):
        assert not LA.allowed_ip(ip), ip


# ---------------------------------------------------------------- the launcher's state (processes stubbed)
class World:
    def __init__(self):
        self.p = dict(runner=[], upload=[], recorder=[], plan=[])
        self.tmux_calls = []
        self.has_session = False

    def procs(self):
        return {k: list(v) for k, v in self.p.items()}

    def tmux(self, *a, **k):
        self.tmux_calls.append(a)
        if a[0] == "has-session":
            return (0 if self.has_session else 1), ""
        return 0, ""


def cams_ok():
    return dict(cameras=[], roles=dict(overhead=dict(ok=True, words="ok", dev=2, name="USB"),
                                       side=dict(ok=True, words="ok", dev=0, name="EMEET")))


def cams_no_overhead():
    r = cams_ok()
    r["roles"]["overhead"] = dict(ok=False, dev=None, name="USB 2.0 Camera",
                                  words="the overhead camera (USB 2.0 Camera) is not plugged in or not found")
    return r


def make_launcher(tmp_path, world, cams=cams_ok, script_body="exit 0"):
    script = tmp_path / "run.sh"
    script.write_text("#!/bin/bash\necho \"args: $*\"\necho \"launcher: $E02_LAUNCHER card: ${E02_CARD_OK:-}\"\n"
                      + script_body + "\n")
    conf = LA.Conf(str(tmp_path / "launcher.json"))
    conf.set_pin("4321")
    (tmp_path / "logs").mkdir(exist_ok=True)
    return LA.Launcher(repo=str(tmp_path), script=str(script), conf=conf, proc_scan=world.procs, tmux_fn=world.tmux,
                       cams_fn=cams, state_dir=str(tmp_path / "state"), ask_fn=lambda ips: ({"Left": 80}, ""))


def wait_for(fn, s=10):
    t = time.time()
    while time.time() - t < s:
        if fn():
            return True
        time.sleep(0.05)
    return False


def test_start_runs_the_script_and_refuses_a_second(tmp_path):
    w = World()
    L = make_launcher(tmp_path, w)
    ok, words = L.start(dict(hours="1", card_ok=True))
    assert ok and "1 hour" in words
    assert L.phase() == "starting"
    ok2, why = L.start({})
    assert not ok2 and "starting already" in why
    assert wait_for(lambda: L.starting is None)
    assert L.last_start["ok"]
    log = open(os.path.join(L.state_dir, L.last_start["log"])).read()
    assert "args: --hours 1 --new" in log and "launcher: 1 card: y" in log
    w.p["runner"] = [123]
    assert L.phase() == "running"
    assert "running" in L.start({})[1]
    w.p = dict(runner=[], upload=[9], recorder=[], plan=[])
    assert L.phase() == "uploading" and "uploading" in L.start({})[1]
    w.p = dict(runner=[], upload=[], recorder=[7], plan=[])
    assert L.phase() == "busy" and "recorder" in L.start({})[1]


def test_start_refused_without_the_overhead_camera(tmp_path):
    L = make_launcher(tmp_path, World(), cams=cams_no_overhead)
    ok, why = L.start({})
    assert not ok and why.startswith("Can't start: the overhead camera") and "not plugged in" in why


def test_failed_start_keeps_what_the_script_said(tmp_path):
    L = make_launcher(tmp_path, World(), script_body='echo "Only 2 GB free; a 2-hour session needs about 7 GB."; exit 1')
    assert L.start({})[0]
    assert wait_for(lambda: L.starting is None)
    assert not L.last_start["ok"] and any("Only 2 GB free" in ln for ln in L.last_start["lines"])
    assert L.grace_until == 0.0


def test_finished_windows_are_closed_before_a_start(tmp_path):
    w = World()
    w.has_session = True                      # e02's windows waiting for Enter; nothing runs in them
    L = make_launcher(tmp_path, w)
    assert L.start({})[0]
    assert ("kill-session", "-t", "e02") in w.tmux_calls


def test_stop_presses_ctrl_c_in_the_runner_window(tmp_path):
    w = World()
    L = make_launcher(tmp_path, w)
    assert not L.stop()[0]                                  # nothing running
    w.p["runner"] = [1]
    ok, words = L.stop()
    assert ok and ("send-keys", "-t", "e02:run", "C-c") in w.tmux_calls
    assert L.phase() == "stopping"
    assert not L.stop()[0]                                  # only once
    w.p = dict(runner=[], upload=[2], recorder=[], plan=[])
    assert L.phase() == "uploading" and not L.stop()[0]     # the upload is never interrupted from here
    w.p = dict(runner=[], upload=[], recorder=[3], plan=[])
    assert L.stop()[0] and ("send-keys", "-t", "e02:rec", "C-c") in w.tmux_calls


def test_status_last_session_unfinished_and_upload(tmp_path):
    w = World()
    L = make_launcher(tmp_path, w)
    d = tmp_path / "logs" / "e02-20261025-211400"
    d.mkdir(parents=True)
    (d / "e02_state.json").write_text(json.dumps(dict(
        schedule=[dict(status="done"), dict(status="done"), dict(status="pending")],
        holds=[{}, {}], session_end=dict(words="ended: no answer for 30 min"))))
    (d / "battery.csv").write_text("t,robot,role,state,still_s,battery\n1,Left,driving,x,0,77\n2,Left,driving,x,0,75\n")
    os.makedirs(L.state_dir, exist_ok=True)
    with open(os.path.join(L.state_dir, "upload.json"), "w") as f:
        json.dump(dict(state="done", session=d.name, link="https://github.com/x/releases/tag/t"), f)
    s = L.status()
    assert s["phase"] == "idle"
    assert s["last"]["runs_done"] == 2 and s["last"]["runs_left"] == 1 and s["last"]["holds"] == 2
    assert s["last"]["end"] == "ended: no answer for 30 min" and s["last"]["upload"]["link"].endswith("/t")
    assert s["unfinished"] == dict(name=d.name, left=1, total=3)
    assert s["bat_last_session"] == {"Left": ["75", "2"]} or s["bat_last_session"] == {"Left": ("75", "2")}
    args, _, _ = LA.build_args(dict(session="resume"), LA.unfinished_session(str(tmp_path)))
    assert args[-2:] == ["--resume", str(d)]


def test_robot_ips_come_from_the_pi_script():
    assert LA.robot_ips(os.path.join(REPO, "run-e02-pi.sh")) == ["192.168.1.250", "192.168.1.252", "192.168.1.253"]


def test_ask_batteries_reads_the_status_line():
    # a stand-in robot on this machine: answers `status` from port 5006 the way the firmware does
    try:
        robot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        robot.bind(("127.0.0.7", LA.ROBOT_CMD_PORT))
    except OSError:
        pytest.skip("port 5006 busy")
    robot.settimeout(5)

    def answer():
        try:
            data, (src, _) = robot.recvfrom(100)
            if data == b"status":
                robot.sendto(b"Left|17|M,status Left (firmware 2026-10-25c): idle, selected E02, battery 83%, "
                             b"WiFi -51 dBm", (src, LA.ROBOT_LOG_PORT))
        except OSError:
            pass
    threading.Thread(target=answer, daemon=True).start()
    try:
        vals, words = LA.ask_batteries(["127.0.0.7", "127.0.0.9"], wait_s=2.0)
    finally:
        robot.close()
    if "in use" in words:
        pytest.skip("port 5005 busy")
    assert vals == {"Left": 83} and "127.0.0.9" in words


# ---------------------------------------------------------------- the web side
@pytest.fixture
def web(tmp_path):
    w = World()
    L = make_launcher(tmp_path, w)
    H = LA.make_handler(L, L.conf)
    L.handler = H
    srv = LA.ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield L, w, "http://127.0.0.1:%d" % srv.server_address[1]
    srv.shutdown()
    srv.server_close()


def req(url, body=None, cookie="", hdr=True):
    r = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                               method="GET" if body is None else "POST")
    if hdr:
        r.add_header("X-Mirlt", "1")
    if cookie:
        r.add_header("Cookie", cookie)
    try:
        with urllib.request.urlopen(r, timeout=5) as x:
            return x.status, json.loads(x.read() or b"{}") if "json" in x.headers.get("Content-Type", "") else {}, \
                x.headers.get("Set-Cookie", "")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}"), ""


def test_pin_protects_everything_but_the_page(web):
    L, w, base = web
    with urllib.request.urlopen(base + "/") as x:
        assert b"Robot sessions" in x.read()
    assert req(base + "/api/status")[0] == 401
    assert req(base + "/api/start", dict(confirm=True))[0] == 401
    assert req(base + "/api/login", dict(pin="0000"))[0] == 401
    code, _, ck = req(base + "/api/login", dict(pin="4321"))
    assert code == 200 and ck.startswith(LA.COOKIE + "=") and "HttpOnly" in ck and "SameSite=Strict" in ck
    cookie = ck.split(";")[0]
    code, s, _ = req(base + "/api/status", cookie=cookie)
    assert code == 200 and s["phase"] == "idle"
    assert req(base + "/api/start", dict(confirm=True), cookie=cookie, hdr=False)[0] == 403   # not from the page
    assert req(base + "/api/start", dict(), cookie=cookie)[0] == 400                          # not confirmed
    w.p["runner"] = [5]
    code, d, _ = req(base + "/api/start", dict(confirm=True), cookie=cookie)
    assert code == 409 and "running" in d["words"]
    code, d, _ = req(base + "/api/check", dict(options={}), cookie=cookie)
    assert not d["ok"] and "running" in d["words"]


def test_wrong_pins_pause(web):
    L, w, base = web
    for _ in range(LA.PIN_FAILS):
        assert req(base + "/api/login", dict(pin="1111"))[0] == 401
    assert req(base + "/api/login", dict(pin="4321"))[0] == 429


def test_a_new_pin_ends_old_logins(tmp_path):
    conf = LA.Conf(str(tmp_path / "l.json"))
    conf.set_pin("1234")
    tok = conf.token()
    assert conf.check_token(tok) and conf.check_pin("1234") and not conf.check_pin("12345")
    conf.set_pin("9999")
    assert not conf.check_token(tok)
    assert oct(os.stat(conf.path).st_mode & 0o777) == "0o600"
    assert "pin_hash" in json.load(open(conf.path)) and "9999" not in open(conf.path).read()


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _listening(port):
    s = socket.socket()
    try:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def test_idle_port_is_handed_over_and_taken_back(tmp_path):
    w = World()
    L = make_launcher(tmp_path, w)
    L.handler = LA.make_handler(L, L.conf)
    port = _free_port()
    L.idle_bind = ("127.0.0.1", port)
    L.tick()
    assert _listening(port)
    L.handoff()                                         # run-e02-pi.sh asks for it
    assert not _listening(port)
    srv = socket.socket()                               # the runner's live page can open it now
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.close()
    L.tick()
    assert not _listening(port)                         # still within the grace: not taken back
    w.p["runner"] = [1]
    L.tick()
    assert L.grace_until == 0.0 and not _listening(port)
    w.p["runner"] = []                                  # the runner ended (the upload runs): back at once
    L.tick()
    assert _listening(port)
    w.p["runner"] = [1]                                 # a runner without a hand-off: let go at once
    L.tick()
    assert not _listening(port)
