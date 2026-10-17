#!/usr/bin/env python3
"""Guided runs: walks you through an experiment's measurement sheet, one run
at a time, and does the bookkeeping.

    python tools/run_sheet.py docs/experiments/E01-baseline-dataset
    python tools/run_sheet.py docs/experiments/E01-baseline-dataset --pi st1maxfini.local

For each row of data/measurements.csv still missing its tape readings, it:
  1. tells you which robot and route, and asks for the START marks
     (front x y, rear x y, in cm), checking them for typos;
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
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alvik_console import Link  # noqa: E402

NOTES_PORT = 5007


def num(x):
    try:
        return float(x)
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


def ask_marks(what, spacing, near=None, max_dist=None, heading_near=None):
    """Ask for 'front_x front_y rear_x rear_y' and check it. Returns the 4 numbers."""
    while True:
        s = ask(f"  {what} marks, cm: front x, front y, rear x, rear y\n  > ")
        parts = s.replace(",", " ").split()
        vals = [num(p) for p in parts]
        if len(vals) != 4 or None in vals:
            print("    Need 4 numbers, e.g.  4.8 0.1 -4.7 0.0   (or q / s / x)")
            continue
        x, y, h, d = pose(*vals)
        warn = []
        if spacing and abs(d - spacing) > 0.5:
            warn.append(f"front-to-rear distance is {d:.1f} cm, expected about {spacing:.1f}")
        if near is not None and max_dist is not None and math.hypot(x - near[0], y - near[1]) > max_dist:
            warn.append(f"position ({x:.1f}, {y:.1f}) is {math.hypot(x - near[0], y - near[1]):.1f} cm from "
                        f"where it should be about ({near[0]:.0f}, {near[1]:.0f})")
        if heading_near is not None and abs(wrap180(h - heading_near)) > 20:
            warn.append(f"heading {h:.0f} deg is far from the expected {heading_near:.0f} deg "
                        f"(front and rear swapped?)")
        if warn:
            print("    Check: " + "; ".join(warn))
            if ask("    Keep these numbers anyway? (y = keep, anything else = re-enter) ").lower() != "y":
                continue
        print(f"    -> position ({x:.1f}, {y:.1f}) cm, heading {h:.1f} deg")
        return vals


def _write_csv(path, cols, rows):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
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
    ap.add_argument("--spacing", type=float, help="distance between the front and rear marks, cm (default: learned from the first entry)")
    ap.add_argument("--run-timeout", type=float, default=180, help="give up waiting for a run to finish after this many seconds")
    args = ap.parse_args()

    sheet = os.path.join(args.experiment, "data", "measurements.csv")
    if not os.path.exists(sheet):
        sys.exit(f"No sheet at {sheet}. Make one with tools/make_run_order.py")
    with open(sheet, newline="") as f:
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

    spacing = args.spacing
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
                    print(f"  {robot} needs its compass check (once per power-on). It will wiggle in place.")
                    ask("  Put it on the floor, NOT on the mark, and press Enter: ")
                    link.send_name(robot, "calibrate")
                    time.sleep(3)
                    calibrated[robot] = watch.boots.get(robot, "?")
                print(f"  Place {robot} at the origin, facing +x. Mark its front and rear, then measure.")
                start = ask_marks("START", spacing, near=(0, 0), max_dist=5, heading_near=0)
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
                if res["result"] != "done":
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
                end = ask_marks("END", spacing, near=(ex, ey), max_dist=20, heading_near=hs + th)
                xe, ye, he, _ = pose(*end)
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
