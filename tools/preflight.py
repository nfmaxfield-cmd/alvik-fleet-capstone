"""Check everything is ready before a recording session, and fetch what can
be fetched ahead of time.

    python tools/preflight.py                      # laptop + robots
    python tools/preflight.py --pi st1maxfini.local
    python tools/preflight.py --robots Left,Center,Right --pi st1maxfini.local

Checks, in order:
  1. Python and the packages each tool needs (says which pip command fixes it)
  2. the comparison detector's COCO-pretrained weights: downloaded now into
     the torch cache, so training later doesn't depend on the internet
  3. the robot port (5005) is free: no other console/run_sheet still running
  4. every robot answers, runs the firmware in this repo, and has battery
  5. the Pi recorder (camrec.py) answers on its notes port, if --pi is given

Nothing here moves a robot.
"""
import argparse
import importlib
import os
import re
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

OK, BAD, WARN = "  ok   ", "  FIX  ", "  note "
problems = []


def say(mark, text, fix=None):
    print(mark + text)
    if mark == BAD:
        problems.append(text + (f"  ->  {fix}" if fix else ""))
    elif fix:
        print("         " + fix)


def repo_fw_version():
    path = os.path.join(HERE, "..", "firmware", "home_lab", "home_lab.ino")
    try:
        with open(path, encoding="utf-8") as f:
            m = re.search(r'#define FW_VERSION "([^"]+)"', f.read())
        return m.group(1) if m else None
    except OSError:
        return None


def check_python():
    print("\n1. Python and packages")
    v = sys.version_info
    if v < (3, 9):
        say(BAD, f"Python {v.major}.{v.minor}", "install Python 3.11 from python.org")
    else:
        say(OK, f"Python {v.major}.{v.minor}.{v.micro}")
        if os.name == "nt" and v < (3, 13):
            say(WARN, "Windows Python before 3.13: the console uses a fine timer itself, nothing to do")
    groups = [
        ("fusion and analysis", ["numpy", "scipy"], "pip install numpy scipy"),
        ("video checks and labelling", ["cv2"], "pip install opencv-python"),
        ("training the detectors", ["torch", "torchvision", "onnx", "onnxruntime"],
         "pip install torch torchvision onnx onnxruntime"),
    ]
    have = {}
    for what, mods, fix in groups:
        missing = []
        for m in mods:
            try:
                importlib.import_module(m)
                have[m] = True
            except Exception:
                missing.append(m)
        if missing:
            say(BAD if what != "training the detectors" else WARN,
                f"{what}: missing {', '.join(missing)}", fix)
        else:
            say(OK, f"{what}: {', '.join(mods)}")
    return have


def fetch_weights(have):
    print("\n2. Comparison detector weights (COCO-pretrained SSDlite)")
    if not (have.get("torch") and have.get("torchvision")):
        say(WARN, "skipped: torch / torchvision not installed (only needed for the detector comparison)")
        return
    try:
        from torchvision.models.detection import (SSDLite320_MobileNet_V3_Large_Weights,
                                                  ssdlite320_mobilenet_v3_large)
        t0 = time.time()
        ssdlite320_mobilenet_v3_large(weights=SSDLite320_MobileNet_V3_Large_Weights.COCO_V1)
        say(OK, f"weights ready in the torch cache ({time.time() - t0:.0f} s)")
    except Exception as e:
        say(BAD, f"could not download the weights: {e}",
            "check the internet connection, or train the comparison with --weights none")


def check_port():
    print("\n3. Robot port")
    from alvik_console import LOG_PORT
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        s.bind(("", LOG_PORT))
        say(OK, f"port {LOG_PORT} is free")
        return True
    except OSError:
        say(BAD, f"port {LOG_PORT} is in use", "close the other console / run_sheet / commtest window")
        return False
    finally:
        s.close()


def check_robots(wanted, listen_s, outdir):
    print("\n4. Robots")
    from alvik_console import Link
    want_fw = repo_fw_version()
    status = {}
    lock = threading.Lock()
    pat = re.compile(r"status (\S+) \(firmware ([^)]+)\): (\w+), selected .*?, battery (\d+)%, WiFi (-?\d+) dBm")

    def listener(name, kind, rtype, vals):
        if kind == "M":
            m = pat.search(vals if isinstance(vals, str) else ",".join(vals))
            if m:
                with lock:
                    status[name] = dict(fw=m.group(2), state=m.group(3), battery=int(m.group(4)),
                                        dbm=int(m.group(5)))

    link = Link(outdir)
    link.session.quiet = True
    link.session.listeners.append(listener)
    try:
        print(f"         listening {listen_s:.0f} s for robots (switch them on now)...")
        end = time.time() + listen_s
        while time.time() < end:
            if wanted and set(wanted) <= set(link.robots()):
                break
            time.sleep(0.5)
        for _ in range(3):
            link.send_all("status")
            time.sleep(1.0)
            with lock:
                if set(link.robots()) <= set(status):
                    break
        names = sorted(set(link.robots()) | set(wanted or []))
        if not names:
            say(BAD, "no robots heard", "switch them on; check they are on the same WiFi as this laptop")
        for n in names:
            st = status.get(n)
            if n not in link.robots():
                say(BAD, f"{n}: not heard", "switch it on, or check its WiFi")
                continue
            if not st:
                say(BAD, f"{n}: heard, but it didn't answer 'status'", "firmware too old? flash home_lab again")
                continue
            line = f"{n}: firmware {st['fw']}, battery {st['battery']}%, WiFi {st['dbm']} dBm, {st['state']}"
            if want_fw and st["fw"] != want_fw:
                say(BAD, line, f"flash firmware {want_fw} (this repo's home_lab)")
            elif st["battery"] < 40:
                say(BAD, line, "charge it: a run on low battery is slower and turns differently")
            elif st["dbm"] < -78:
                say(WARN, line, "weak WiFi: move the router or laptop closer, or expect lost packets")
            else:
                say(OK, line)
    finally:
        link.close()


def check_pi(host):
    print("\n5. Pi recorder")
    from run_sheet import PiNotes
    p = PiNotes(host)
    if p.addr is None:
        say(BAD, f"can't find {host}", "is the Pi on and on this WiFi? try its IP address instead")
    elif p.send("preflight check from the laptop"):
        say(OK, f"camrec.py on {host} answered (the note is in its events.csv)")
    else:
        say(BAD, f"{host} found, but camrec.py didn't answer", "on the Pi: python3 tools/pi/camrec.py --label <name>")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", help="names that must answer, e.g. Left,Center,Right")
    ap.add_argument("--pi", help="Pi hostname or IP running camrec.py")
    ap.add_argument("--listen", type=float, default=12, help="seconds to wait for robots")
    ap.add_argument("--no-weights", action="store_true", help="skip the weights download")
    ap.add_argument("--no-robots", action="store_true", help="skip the robot checks")
    a = ap.parse_args()
    have = check_python()
    if not a.no_weights:
        fetch_weights(have)
    port_free = check_port()
    if not a.no_robots and port_free:
        outdir = os.path.join("logs", "preflight-" + time.strftime("%Y%m%d-%H%M%S"))
        check_robots([r.strip() for r in a.robots.split(",")] if a.robots else [], a.listen, outdir)
    if a.pi:
        check_pi(a.pi)
    print("\nRESULT")
    if problems:
        for p in problems:
            print("  - " + p)
        return 1
    print("  ready")
    return 0


if __name__ == "__main__":
    sys.exit(main())
