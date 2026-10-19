#!/usr/bin/env python3
"""Guided runs: walks you through an experiment's measurement sheet, one run
at a time, and does the bookkeeping.

    python tools/run_sheet.py docs/experiments/E01-baseline-dataset
    python tools/run_sheet.py docs/experiments/E01-baseline-dataset --pi st1maxfini.local

For each row of data/measurements.csv still missing its tape readings, it:
  1. tells you which robot and route, and asks for the START marks
     (front x y, rear x y, in cm), checking them for typos; press Enter
     instead to measure them after the run (the dots stay on the floor);
  2. sends that row's commands to the robot, notes the run id and, with --pi,
     marks the moment in the camera recording;
  3. waits for the run to finish (or for you to press Ctrl+C to stop the robot);
  4. asks for the END marks, checks them, shows the error at once;
  5. saves the row (battery and run id filled in automatically).

The sheet is saved after every run, so you can stop any time (type q) and
start again later: it picks up at the first row without tape readings.
Everything the robots send is logged to logs/<date-time>/ exactly like the
console, so check_session.py and tape_results.py work on it.

At any prompt: q = quit (saves), s = skip this run for now, x = exclude it
(asks why). If a check flags your numbers, answer anything but y to re-enter them.

Needs the robots running firmware 2026-10-17b or later.
"""
import argparse
import csv
import datetime as dt
import math
import os
import re
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alvik_console import Link  # noqa: E402
import session_checks as sc  # noqa: E402

NOTES_PORT = 5007


def num(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def pose(xf, yf, xr, yr):
    """Midpoint and heading (rear -> front) of two marks."""
    return (xf + xr) / 2, (yf + yr) / 2, math.degrees(math.atan2(yf - yr, xf - xr)), math.hypot(xf - xr, yf - yr)


class Quit(Exception):
    pass


class Skip(Exception):
    pass


class Exclude(Exception):
    pass


class RunWatcher:
    """Collects what the robots report about their runs."""

    def __init__(self):
        self.cv = threading.Condition()
        self.started = {}    # robot -> (run id, experiment) from its latest RUNINFO
        self.ended = {}      # run id -> dict(result, battery)
        self.demo_end = {}   # run id -> robot's own return error (cm)
        self.straight = {}   # run id -> robot's own end (x, y)
        self.boots = {}      # robot -> boot id seen in clock answers
        self.params = {}     # run id -> {param name: value} from its PARAM records
        self.looks = {}      # run id -> its CALIB_LOOK record (calibration runs, firmware 2026-10-18c+)
        self.messages = []

    def __call__(self, name, kind, rtype, vals):
        with self.cv:
            if kind == "M":
                self.messages.append((time.time(), name, vals))
            elif rtype == "RUNINFO" and len(vals) >= 2:
                self.started[name] = (vals[0], vals[1])
            elif rtype == "RUNEND" and len(vals) >= 5:
                self.ended[vals[0]] = dict(result=vals[1], battery=vals[4])
            elif rtype == "DEMO_END" and len(vals) >= 5:
                self.demo_end[vals[0]] = num(vals[4])
            elif rtype == "STRAIGHT" and len(vals) >= 7:
                self.straight[vals[0]] = (num(vals[5]), num(vals[6]))
            elif rtype == "CLOCK" and len(vals) >= 3:
                self.boots[name] = vals[2]
            elif rtype == "CALIB_LOOK" and len(vals) >= 10:
                self.looks[vals[0]] = dict(heading=num(vals[3]), room=num(vals[5]), roll=num(vals[8]), result=vals[9])
                if len(vals) >= 15:          # spin, roll, reason: what it skipped for lack of room
                    self.looks[vals[0]].update(spin=vals[12], roll_step=vals[13], reason=vals[14])
            elif rtype == "PARAM" and len(vals) >= 3:
                self.params.setdefault(vals[0], {})[vals[1]] = num(vals[2])
            self.cv.notify_all()

    def wait_start(self, robot, before, timeout):
        """Wait for a RUNINFO from robot that is newer than `before`."""
        end = time.time() + timeout
        with self.cv:
            while time.time() < end:
                cur = self.started.get(robot)
                if cur and cur != before:
                    return cur[0]
                self.cv.wait(0.2)
        return None

    def wait_end(self, run, timeout, poll=None):
        end = time.time() + timeout
        with self.cv:
            while time.time() < end:
                if run in self.ended:
                    return self.ended[run]
                self.cv.wait(0.2)
                if poll:
                    poll()
        return None


def ask(prompt):
    try:
        s = input(prompt).strip()
    except EOFError:
        raise Quit()
    low = s.lower()
    if low == "q":
        raise Quit()
    if low == "s":
        raise Skip()
    if low == "x":
        raise Exclude()
    return s


SPACING_TOL = 0.3   # cm: T0 measured the front-to-rear spacing to SD 0.08 cm; 1/8" off is 0.32 cm
INCHES = False   # --inches: tape entries are typed in inches and converted to cm


_INCH_RE = re.compile(r"^(-)?\s*(?:(\d+(?:\.\d+)?)(?:\s*[\s-]\s*(\d+)\s*/\s*(\d+))?|(\d+)\s*/\s*(\d+))$")


def inches(text):
    """'36 3/32', '36-3/32', '36.09', '3/8', '-1 1/2' -> inches as a float.
    Anything else (two whole numbers, a fraction of 1 or more, a fraction
    first, nan) -> None, so a typo is asked again instead of saved."""
    t = text.strip().replace('"', "").strip()
    if t.lower().endswith("in"):
        t = t[:-2].strip()
    m = _INCH_RE.match(t)
    if not m:
        return None
    neg, whole, a, b, fa, fb = m.groups()
    if whole is not None:
        v = float(whole)
        if a is not None:
            if int(b) == 0 or int(a) >= int(b):
                return None
            v += int(a) / int(b)
    else:
        if int(fb) == 0 or int(fa) >= int(fb):
            return None
        v = int(fa) / int(fb)
    return -v if neg else v


# The calibration run's moves with the firmware defaults (params.h, 2026-10-18c): it spins
# calib_spin_deg each way first, looking around with its front sensor, then turns to the
# direction with the most room and rolls up to calib_roll_cm forward and back. Space it needs, cm.
CALIB_ROLL_CM = 15.0      # calib_roll_cm
CALIB_STOP_CM = 7.0       # safe_front_cm: it stops if anything is this close in front, even at the end of the roll
CALIB_MARGIN_CM = 3.0     # calib_margin_cm: it plans to end the roll this much further from anything than that
ROBOT_LEN_CM = 10.0       # the robot's own length (9.5 cm), rounded up
SPIN_CIRCLE_CM = 25.0     # spin in place: corners sweep a ~13.5 cm circle; 25 cm leaves room for drift
EDGE_CM = 25.0            # open plywood edge: roll (15) + half the robot (5) + 5 cm to spare
CALIB_S = 65              # how long it takes, about (62-65 s in the simulator with a roll, settle_end_ms included)


def calib_space_text():
    """The calibration prompt's space instructions, in cm and inches."""
    lane = ROBOT_LEN_CM + CALIB_ROLL_CM + CALIB_STOP_CM + CALIB_MARGIN_CM
    return (f"  It needs a clear circle {SPIN_CIRCLE_CM:g} cm ({to_in(SPIN_CIRCLE_CM)}) across around where it stands, "
            f"to spin.\n"
            f"  It picks its own direction to roll: the one where its front sensor saw the most room. It rolls the "
            f"full {CALIB_ROLL_CM:g} cm when some direction has {lane:g} cm ({to_in(lane)}) of clear floor, counted "
            f"from its back (its own {ROBOT_LEN_CM:g} cm, the roll, and {CALIB_STOP_CM + CALIB_MARGIN_CM:g} cm to spare); "
            f"less room, a shorter roll; under {ROBOT_LEN_CM + 5 + CALIB_STOP_CM + CALIB_MARGIN_CM:g} cm "
            f"({to_in(ROBOT_LEN_CM + 5 + CALIB_STOP_CM + CALIB_MARGIN_CM)}) every way, no roll.\n"
            f"  Keep it at least {EDGE_CM:g} cm ({to_in(EDGE_CM)}) from any open plywood edge (an edge with no board "
            f"along it): the front sensor can't see an edge, so open space past it looks like room.\n"
            f"  Nothing else may move inside that space (hands, other robots).")


def to_in(cm):
    """cm -> inches to the nearest 1/16, written like 15 3/4 (for messages)."""
    n = round(abs(cm) / 2.54 * 16)
    w, f = divmod(n, 16)
    g = math.gcd(f, 16)
    frac = f" {f // g}/{16 // g}" if f else ""
    return f"{'-' if cm < 0 and n else ''}{w}{frac}\""


def where(x, y):
    """A floor position for messages: cm, plus inches in --inches mode."""
    return f"({x:.1f}, {y:.1f}) cm" + (f" = ({to_in(x)}, {to_in(y)})" if INCHES else "")


def parse_marks(s):
    """Four entries. In cm: spaces or commas between them. In inches (--inches):
    commas between them, each like 36 3/32 (whole inches and a fraction)."""
    if INCHES:
        parts = [p for p in s.split(",") if p.strip()]
        vals = [inches(p) for p in parts]
        return [None if v is None else round(v * 2.54, 2) for v in vals]
    return [num(p) for p in s.replace(",", " ").split()]


def ask_marks(what, spacing, near=None, max_dist=None, heading_near=None, heading_tol=20, allow_later=False):
    """Ask for 'front_x front_y rear_x rear_y' and check it. Returns the 4 numbers (cm),
    or None if allow_later and the user pressed Enter (measure after the run)."""
    while True:
        unit = 'inches, comma between each: front x, front y, rear x, rear y' if INCHES else \
            "cm: front x, front y, rear x, rear y"
        later = "\n  (or just press Enter to measure them after the run)" if allow_later else ""
        s = ask(f"  {what} marks, {unit}{later}\n  > ")
        if allow_later and not s.strip():
            print("    OK: measure these start marks after the run. Leave the dots on the floor.")
            return None
        vals = parse_marks(s)
        if len(vals) != 4 or None in vals:
            print("    Need 4 entries, e.g.  " + ("36 3/32, 42 15/16, 39 27/32, 42 13/16" if INCHES
                                                 else "4.8 0.1 -4.7 0.0") + "   (or q / s / x)")
            continue
        if INCHES:
            print("    = " + ", ".join(f"{v:.2f}" for v in vals) + " cm")
        x, y, h, d = pose(*vals)
        warn = []
        if spacing and abs(d - spacing) > SPACING_TOL:
            warn.append(f"front-to-rear distance is {d:.1f} cm, expected about {spacing:.1f}")
        if near is not None and max_dist is not None and math.hypot(x - near[0], y - near[1]) > max_dist:
            warn.append(f"position {where(x, y)} is {math.hypot(x - near[0], y - near[1]):.1f} cm from "
                        f"where it should be about {where(near[0], near[1])}")
        if heading_near is not None and abs(wrap180(h - heading_near)) > heading_tol:
            warn.append(f"heading {h:.0f} deg is {abs(wrap180(h - heading_near)):.0f} deg from the expected "
                        f"{heading_near:.0f} deg (a misread inch, or front and rear swapped?)")
        if warn:
            print("    Check: " + "; ".join(warn))
            if ask("    Keep these numbers anyway? (y = keep, anything else = re-enter) ").lower() != "y":
                continue
        print(f"    -> position {where(x, y)}, heading {h:.1f} deg")
        return vals


def _write_csv(path, cols, rows):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:     # plain UTF-8 (no BOM)
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)


def check_params(watch, run, cmds, wait_s=1.5):
    """Compare 'set name value' commands with the run's PARAM records.
    Returns (mismatches, names not reported)."""
    wanted = {}
    for c in cmds:
        p = c.split()
        if len(p) == 3 and p[0] == "set" and num(p[2]) is not None:
            wanted[p[1]] = num(p[2])
    if not wanted:
        return [], []
    end = time.time() + wait_s
    with watch.cv:
        while time.time() < end and not set(wanted) <= set(watch.params.get(run, {})):
            watch.cv.wait(0.1)
        got = dict(watch.params.get(run, {}))
    bad = [f"{k} is {got[k]:g}, not {v:g}" for k, v in wanted.items() if k in got and abs(got[k] - v) > 1e-3]
    unseen = [k for k in wanted if k not in got]
    return bad, unseen


# Filled in by a run: a replacement row starts without them.
RUN_FILLED_COLS = ("run_id", "xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm", "xf_cm", "yf_cm", "xr_cm", "yr_cm",
                   "battery_pct", "excluded", "note")


def add_replacement(rows, i):
    """A limited run (edge guard: limited:plywood edge) is left out of the statistics, so its level would
    lose a run: append a copy of rows[i] (same robot, level, commands, targets) at the end of that robot's
    block (the stretch of consecutive rows of the same robot that row i is in), with a new order number
    and the note "replaces order N (limited: plywood edge)". Returns the new row."""
    r = rows[i]
    j = i
    while j + 1 < len(rows) and rows[j + 1].get("robot") == r.get("robot"):
        j += 1
    orders = [int(float(x["order"])) for x in rows if str(x.get("order", "")).strip().lstrip("-").replace(".", "", 1).isdigit()]
    new = {k: ("" if k in RUN_FILLED_COLS else v) for k, v in r.items()}
    new["order"] = str(max(orders, default=0) + 1)
    new["note"] = f"replaces order {r.get('order')} ({sc.LIMITED_EDGE_NOTE})"
    rows.insert(j + 1, new)
    return new


def save_sheet(path, cols, rows, interactive=True):
    """Save atomically. Windows refuses to replace a file Excel has open, so a
    failed save never loses the marks: they always go to a backup copy first,
    and the user is asked to close Excel and retry."""
    backup = path + ".backup.csv"
    try:
        _write_csv(backup, cols, rows)
    except OSError as e:
        print(f"    (could not write the backup {backup}: {e})")
    while True:
        try:
            _write_csv(path, cols, rows)
            return True
        except OSError as e:
            print(f"\n  Could not save {path}: {e}")
            print(f"  (Is it open in Excel? Everything is safe in {backup}.)")
            if not interactive:
                return False
            try:
                if input("  Close it, then press Enter to retry (or type skip): ").strip().lower() == "skip":
                    return False
            except (EOFError, KeyboardInterrupt):
                return False


class PiNotes:
    """Sends notes to the Pi recorder (camrec.py) and checks it heard them."""

    def __init__(self, host):
        self.host = host
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        try:
            self.addr = (socket.gethostbyname(host), NOTES_PORT)
        except OSError:
            print(f"  (can't find the Pi at {host}; notes won't be sent to the recording)")
            self.addr = None

    def send(self, text):
        if not self.addr:
            return False
        for _ in range(3):
            try:
                self.sock.sendto(text.encode(), self.addr)
                data, _ = self.sock.recvfrom(512)
                if data.startswith(b"ok "):
                    return True
            except OSError:
                pass
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("experiment", help="experiment folder, e.g. docs/experiments/E01-baseline-dataset")
    ap.add_argument("--pi", help="Pi hostname running camrec.py, to mark each run in the video (e.g. st1maxfini.local)")
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    ap.add_argument("--outdir", help="log folder (default logs/<date-time>)")
    ap.add_argument("--inches", action="store_true", help="type tape measurements in inches (e.g. 36 3/32); saved in cm")
    ap.add_argument("--spacing", type=float, default=9.5,
                    help="distance between the front and rear marks, cm (T0 measured 9.52; 0 = learn from the first entry)")
    ap.add_argument("--run-timeout", type=float, default=180, help="give up waiting for a run to finish after this many seconds")
    args = ap.parse_args()
    global INCHES
    INCHES = args.inches

    sheet = os.path.join(args.experiment, "data", "measurements.csv")
    if not os.path.exists(sheet):
        sys.exit(f"No sheet at {sheet}. Make one with tools/make_run_order.py")
    with open(sheet, newline="", encoding="utf-8-sig") as f:   # -sig: Excel's "CSV UTF-8" starts with a BOM
        reader = csv.DictReader(f)
        cols = reader.fieldnames
        rows = list(reader)

    def done(r):
        return (r.get("excluded") or "").strip().lower() in ("1", "yes", "y", "x", "true") or all(
            (r.get(k) or "").strip() for k in ("xf_cm", "yf_cm", "xr_cm", "yr_cm"))
    todo = [r for r in rows if not done(r)]
    print(f"{len(rows)} runs on the sheet, {len(rows) - len(todo)} done, {len(todo)} to go.")
    if not todo:
        return 0

    outdir = args.outdir or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S"))
    link = Link(outdir, args.robot)
    link.session.quiet = True
    watch = RunWatcher()
    link.session.listeners.append(watch)
    pi = PiNotes(args.pi) if args.pi else None
    log_path = os.path.join(outdir, "run_sheet_log.csv")
    os.makedirs(outdir, exist_ok=True)
    new_log = not os.path.exists(log_path)
    logf = open(log_path, "a", newline="")
    logw = csv.writer(logf)
    if new_log:
        logw.writerow(["t", "order", "robot", "level", "run_id", "result", "event"])

    spacing = args.spacing or None
    calibrated = {}   # robot -> boot id it was calibrated in
    needed = sorted({r["robot"] for r in todo})
    print(f"Logging to {outdir}. Waiting for robots: {', '.join(needed)} ...")
    t0 = time.time()
    while time.time() - t0 < 15 and not set(needed) <= set(link.robots()):
        time.sleep(0.5)
    print(f"Connected: {', '.join(link.robots()) or 'none'}.")
    missing = set(needed) - set(link.robots())
    if missing:
        print(f"Not heard from: {', '.join(sorted(missing))}. Their runs will wait until they're on.")
    time.sleep(2.5)   # first clock checks report each robot's boot id

    last = None
    try:
        i = 0
        while i < len(rows):
            r = rows[i]
            if done(r):
                i += 1
                continue
            robot, level = r["robot"], r["level"]
            n_done = sum(1 for x in rows if done(x))
            print(f"\n=== Run {r['order']} ({n_done + 1} of {len(rows)}): {robot}, {level} ===")
            try:
                if robot not in link.robots():
                    print(f"  {robot} isn't connected. Switch it on (s = skip for now).")
                    while robot not in link.robots():
                        ask("  Press Enter once it's on: ")
                boot = watch.boots.get(robot)
                if calibrated.get(robot) is None or (boot and calibrated[robot] != boot):
                    print(f"  {robot} needs its calibration run (once per power-on, about {CALIB_S} s): it blinks "
                          f"its code, wiggles, spins slowly twice around to the left and back to the right while it "
                          f"looks around, then turns to where it saw the most room and rolls up to {CALIB_ROLL_CM:g} cm "
                          f"({to_in(CALIB_ROLL_CM)}) forward and back.")
                    print(calib_space_text())
                    ask("  Put it on the floor where the camera can see all of that space, NOT on the mark, facing "
                        "any way, and press Enter: ")
                    before = watch.started.get(robot)
                    link.send_name(robot, "calibrate")
                    crun = watch.wait_start(robot, before, 8)
                    cres = watch.wait_end(crun, CALIB_S + 50) if crun else None
                    look = watch.looks.get(crun)
                    outcome = sc.calib_outcome(cres["result"] if cres else None, look)
                    if outcome == sc.CALIB_FAILED:
                        why = cres["result"] if cres else "no answer"
                        print(f"  Calibration didn't finish ({why}). Check the robot has room, then try again.")
                        continue
                    if outcome == sc.CALIB_LIMITED:
                        # not a failure: it did what the space allowed; nothing is counted against it
                        no_spin = look.get("spin") == "skipped"
                        edge = look.get("result") == "edge"
                        edge_kept = look.get("roll_step") == "edge"
                        no_roll = look.get("roll_step") == "skipped" or ("roll_step" not in look and
                                                                          (look["roll"] or 0) <= 0)
                        if edge_kept:
                            print(f"  Calibrated with a shorter roll: the edge guard stopped it at a plywood edge after "
                                  f"{look['roll']:.1f} cm (the look-around can't see an open edge). That roll is kept"
                                  + ("; no spin either (something inside its 25 cm circle)." if no_spin else "."))
                        elif edge and no_roll:
                            print("  Calibrated without the roll: the edge guard stopped it at a plywood edge too soon "
                                  "to measure. " + ("Spin results are kept; " if not no_spin else "")
                                  + "heading and wheel size come from the camera and later runs instead.")
                        elif no_roll and not no_spin:
                            print("  Calibrated without the roll: no room in any direction. Spin results are kept; "
                                  "heading and wheel size come from the camera and later runs instead.")
                        elif no_spin and not no_roll:
                            print("  Calibrated without the spin: something is inside the 25 cm circle it needs. Roll "
                                  "results are kept; gyro scale and light offset come from a later calibration instead.")
                        else:
                            print("  Calibrated without the spin or the roll: no room (something inside its 25 cm "
                                  "circle, and no room to roll in any direction). Its blink code and compass check are "
                                  "kept; heading, wheel size, gyro scale and light offset come from the camera, later "
                                  "runs and a later calibration instead.")
                        if ask("  Move it and press r to redo, or Enter to continue: ").lower() == "r":
                            continue
                        print(f"  Calibrated, {sc.calib_outcome_text(cres['result'], look)} (run {crun}).")
                    else:
                        if look and look["result"] == "short":
                            print(f"  (It rolled {look['roll']:.1f} cm, not the full {CALIB_ROLL_CM:g}: not much room.)")
                        print(f"  Calibrated (run {crun}).")
                    if pi:
                        pi.send(f"calibration {robot} run {crun}")
                    calibrated[robot] = watch.boots.get(robot, "?")
                sx, sy, sh = num(r.get("start_x_cm")), num(r.get("start_y_cm")), num(r.get("start_heading_deg"))
                if sx is None or sy is None:
                    sx, sy = 0.0, 0.0
                sh = 0.0 if sh is None else sh
                spot = "the origin" if (sx, sy) == (0.0, 0.0) else (
                    f"x = {sx:g} cm, y = {sy:g} cm" + (f" (x = {to_in(sx)}, y = {to_in(sy)})" if INCHES else ""))
                print(f"  Place {robot} with its centre at {spot}, facing {'+x' if sh == 0 else f'{sh:g} deg'}. "
                      f"Mark its front and rear, then measure.")
                start = ask_marks("START", spacing, near=(sx, sy), max_dist=5, heading_near=sh, heading_tol=5,
                                  allow_later=True)
                if start is None:            # measured after the run: use the planned spot until then
                    xs, ys, hs = sx, sy, sh
                else:
                    xs, ys, hs, d = pose(*start)
                    if spacing is None:
                        spacing = round(d, 1)
                        print(f"    (front-to-rear mark distance taken as {spacing} cm for the checks)")
                cmds = [c.strip() for c in r["commands"].split("/") if c.strip()]
                ask(f"  Ready to send: {' / '.join(cmds)}  - press Enter to start: ")
                before = watch.started.get(robot)
                for c in cmds:
                    link.send_name(robot, c)
                    time.sleep(0.2)
                run = watch.wait_start(robot, before, 6)
                if not run:
                    with watch.cv:
                        said = [v for t, n, v in watch.messages if n == robot and time.time() - t < 10]
                    for v in said[-3:]:
                        print(f"  {robot} says: {v if isinstance(v, str) else ','.join(v)}")
                    if any("not calibrated" in (v if isinstance(v, str) else ",".join(v)) for v in said):
                        calibrated.pop(robot, None)     # it rebooted: check the compass again next time
                        print("  It restarted since its compass check. Redoing this trial with the check first.")
                        continue
                    print("  The robot didn't report a run start. Is it busy, or out of WiFi range? (s = skip)")
                    raise Skip()
                print(f"  Running: {run}  (Ctrl+C stops the robot)")
                # Each "set" went out as one WiFi packet with no reply; a lost one
                # would leave the robot on its old value. Check the robot's own
                # PARAM records against what the sheet asked for.
                bad, unseen = check_params(watch, run, cmds)
                if bad:
                    link.send_name(robot, "stop")
                    print("  The robot didn't get every setting (a WiFi packet was lost): " + "; ".join(bad))
                    print("  Stopped it. Put it back on the start mark; the trial will be redone.")
                    watch.wait_end(run, 5)
                    r["run_id"] = ""
                    continue
                if unseen:
                    print(f"  (couldn't confirm {', '.join(unseen)}: their PARAM records didn't arrive; "
                          f"check_session will show the values)")
                note = f"trial {r['order']} {robot} {level} run {run} start"
                if pi:
                    print("  Marked in video." if pi.send(note) else "  (The Pi recorder didn't answer: type the note there.)")
                logw.writerow([f"{time.time():.3f}", r["order"], robot, level, run, "", "start"])
                logf.flush()
                try:
                    res = watch.wait_end(run, args.run_timeout)
                except KeyboardInterrupt:
                    link.send_name(robot, "stop")
                    print("\n  Stop sent.")
                    res = watch.wait_end(run, 5)
                if res is None:
                    print("  No end record from the robot. Treating the run as unfinished.")
                    res = dict(result="no end record", battery="")
                print(f"  Finished: {res['result']}, battery {res.get('battery', '?')}%")
                if pi:
                    pi.send(f"trial {r['order']} run {run} end {res['result']}")
                logw.writerow([f"{time.time():.3f}", r["order"], robot, level, run, res["result"], "end"])
                logf.flush()
                r["run_id"] = run
                r["battery_pct"] = res.get("battery", "")
                if sc.run_outcome(res["result"]) == sc.CALIB_LIMITED:
                    # the edge guard refused to drive off the work area: not a localization failure, but the
                    # run didn't finish its route, so accuracy statistics leave it out (counted apart)
                    print("  The edge guard stopped it before an open plywood edge. That is not a failure, but the "
                          "run didn't finish its route, so the accuracy statistics should leave it out. Unless you "
                          "redo it now, a replacement run (same robot and level) is added at the end of this "
                          f"robot's block, so {level} keeps its planned number of runs.")
                    a = ask(f"  Press Enter to mark it excluded with the note \"{sc.LIMITED_EDGE_NOTE}\", r to redo "
                            f"it, or m to measure it anyway: ").lower()
                    if a == "r":
                        r["run_id"] = ""
                        continue
                    rep = add_replacement(rows, i)
                    print(f"  Added run {rep['order']} to replace it ({robot}, {level}).")
                    if a != "m":
                        r["excluded"] = "1"
                        r["note"] = (r.get("note", "") + " " + sc.LIMITED_EDGE_NOTE).strip()
                        save_sheet(sheet, cols, rows)
                        print("  Excluded (limited: plywood edge) and saved.")
                        i += 1
                        continue
                    # measured for the record: the note still keeps it out of the accuracy statistics
                    r["note"] = (r.get("note", "") + " " + sc.LIMITED_EDGE_NOTE + " (measured anyway)").strip()
                elif res["result"] != "done":
                    a = ask("  The run didn't finish normally. Measure it anyway (y), redo it (r) or exclude it (x)? ").lower()
                    if a == "r":
                        r["run_id"] = ""
                        continue
                    if a != "y":
                        raise Exclude()
                tx, ty, th = (num(r.get("target_x_cm")) or 0.0, num(r.get("target_y_cm")) or 0.0,
                              num(r.get("target_heading_deg")) or 0.0)
                c, s = math.cos(math.radians(hs)), math.sin(math.radians(hs))
                ex, ey = xs + c * tx - s * ty, ys + s * tx + c * ty
                end = ask_marks("END", spacing, near=(ex, ey), max_dist=20, heading_near=hs + th, heading_tol=10)
                xe, ye, he, _ = pose(*end)
                # when the end marks were typed: the robot must not have moved before this (accuracy check B14)
                logw.writerow([f"{time.time():.3f}", r["order"], robot, level, run, res["result"], "end_marks"])
                logf.flush()
                if pi:
                    pi.send(f"trial {r['order']} run {run} final marks typed")
                if start is None:
                    print("  Now the START marks you made before the run (lift the robot away first if it's in the way).")
                    start = ask_marks("START", spacing, near=(sx, sy), max_dist=5, heading_near=sh, heading_tol=5)
                    xs, ys, hs, _ = pose(*start)
                    c, s = math.cos(math.radians(hs)), math.sin(math.radians(hs))
                    ex, ey = xs + c * tx - s * ty, ys + s * tx + c * ty
                for k, v in zip(("xf0_cm", "yf0_cm", "xr0_cm", "yr0_cm"), start):
                    r[k] = f"{v:g}"
                for k, v in zip(("xf_cm", "yf_cm", "xr_cm", "yr_cm"), end):
                    r[k] = f"{v:g}"
                save_sheet(sheet, cols, rows)
                dx, dy = xe - ex, ye - ey
                own = watch.demo_end.get(run)
                if own is None and run in watch.straight and watch.straight[run][0] is not None:
                    ox, oy = watch.straight[run]
                    own = math.hypot(ox - tx, oy - ty)
                msg = (f"  Saved. Tape: ended {math.hypot(dx, dy):.1f} cm from target "
                       f"({c * dx + s * dy:+.1f} along, {-s * dx + c * dy:+.1f} left), heading "
                       f"{wrap180(he - (hs + th)):+.1f} deg")
                if own is not None:
                    msg += f". Robot thought: {own:.1f} cm"
                print(msg)
                last = i
                i += 1
            except Skip:
                print("  Skipped for now.")
                i += 1
            except Exclude:
                why = ""
                try:
                    why = input("  Why exclude it? ").strip()
                except EOFError:
                    pass
                r["excluded"] = "1"
                r["note"] = (r.get("note", "") + " " + why).strip()
                save_sheet(sheet, cols, rows)
                print("  Excluded and saved.")
                i += 1
    except (Quit, KeyboardInterrupt):
        print("\nStopping.")
    finally:
        link.close()             # stops the robots first, whatever happens next
        logf.close()
        save_sheet(sheet, cols, rows)
        left = sum(1 for x in rows if not done(x))
        print(f"\nSheet saved: {sheet} ({len(rows) - left} done, {left} to go).")
        print(f"Robot logs: {outdir}")
        for n, got, dropped, loss in link.summary():
            print(f"  {n}: {got} packets, {dropped} dropped ({loss:.1f}%)")
        print(f"Next: python tools/check_session.py {outdir} --video <recording folder>")
    return 0


if __name__ == "__main__":
    sys.exit(main())
