#!/usr/bin/env python3
"""A quick recording session for the recognition studio: every robot drives short random
routes at the same time, then you put objects on and off the board, then it ends.

    python tools/roam_session.py --pi st1maxfini.local
    python tools/roam_session.py --pi st1maxfini.local --minutes 4 --robots Left,Center,Right

Start the Pi recorder first (one or two cameras). What happens:
  1. Waits until the robots are connected (switch them on, put them on the board at least
     30 cm apart and 35 cm from the edges).
  2. Calibration run on every robot at once (about a minute: they blink, spin and roll).
     This also does the compass check the routes need.
  3. Roaming for --minutes (default 4): each robot drives one short route after another,
     chosen at random (an L, a circle, a small square, with random sizes), so the camera
     sees every robot from many angles, close together and apart. A robot that meets
     another one stops by itself (front stop) and starts its next route.
     If a robot gets near an edge, pick it up and put it back near the middle: your hand
     in the picture is useful too.
  4. Objects: it tells you to put things on and off the board, by hand, for as long as
     you like; press Enter when done.
  5. Stops every robot. Each phase is marked in the recording as a note.

Ctrl+C at any time stops every robot. The robot logs go to logs/<date-time>/ as usual.
"""
import argparse
import datetime as dt
import os
import random
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from alvik_console import Link  # noqa: E402
from run_sheet import PiNotes, RunWatcher  # noqa: E402

ROUTES = ("L", "circle", "square")


def route_commands(rng):
    """One random short route: the settings to send, then `run demo`. Every route ends near
    where it started, so the robots stay on the board (sizes kept small on purpose)."""
    kind = rng.choice(ROUTES)
    if kind == "L":
        a, b = rng.randint(15, 30), rng.randint(10, 25)
        return kind, ["set demo_route 1", "set demo_a_cm %d" % a, "set demo_b_cm %d" % b], "L %d/%d cm" % (a, b)
    if kind == "circle":
        a, r = rng.randint(5, 15), rng.randint(10, 18)
        return kind, ["set demo_route 2", "set demo_a_cm %d" % a, "set demo_r_cm %d" % r], "circle r %d cm" % r
    s = rng.randint(15, 30)
    return kind, ["set demo_route 3", "set square_cm %d" % s, "set square_laps 1"], "square %d cm" % s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", help="the Pi recorder's address, for marking phases in the video (e.g. st1maxfini.local)")
    ap.add_argument("--robots", default="Left,Center,Right")
    ap.add_argument("--minutes", type=float, default=4.0, help="how long the robots roam (default 4)")
    ap.add_argument("--no-calibrate", action="store_true", help="skip the calibration runs (only if done since power-on)")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--robot", action="append", default=[], help="robot IP, if robots aren't found automatically")
    a = ap.parse_args()

    names = [n.strip() for n in a.robots.split(",") if n.strip()]
    rng = random.Random(a.seed)
    outdir = os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "_roam")
    link = Link(outdir, a.robot)
    watch = RunWatcher()
    link.session.listeners.append(watch)
    pi = PiNotes(a.pi) if a.pi else None

    def note(text):
        print("  [mark] " + text)
        if pi and not pi.send(text):
            print("  (the Pi recorder didn't answer; type the note there if you want it marked)")

    def stop_all():
        for _ in range(3):
            link.send_all("stop")
            time.sleep(0.1)

    try:
        print("Logging to %s. Waiting for %s ... (switch them on)" % (outdir, ", ".join(names)))
        t0 = time.time()
        while True:
            have = set(link.robots())
            if all(n in have for n in names):
                break
            if time.time() - t0 > 60:
                missing = [n for n in names if n not in have]
                print("Still missing: %s. Going on with %s." % (", ".join(missing), ", ".join(sorted(have)) or "none"))
                names = [n for n in names if n in have]
                break
            time.sleep(0.5)
        if not names:
            sys.exit("No robots connected.")
        input("\nPut %s on the board, at least 30 cm apart and 35 cm from the edges. Press Enter to start ... "
              % ", ".join(names))

        if not a.no_calibrate:
            note("phase calibration")
            print("Calibration runs (about a minute; they blink, spin and roll) ...")
            before = {n: watch.started.get(n) for n in names}
            for n in names:
                link.send_name(n, "calibrate")
            runs = {n: watch.wait_start(n, before[n], 15) for n in names}
            for n, run in runs.items():
                if not run:
                    print("  %s did not start its calibration run (check it's on and not busy)" % n)
                    continue
                end = watch.wait_end(run, 150)
                print("  %s: %s" % (n, end["result"] if end else "no end report (still going?)"))

        note("phase roaming")
        print("\nRoaming for %.1f minutes. Put a robot back near the middle if it gets near an edge." % a.minutes)
        stop_at = time.time() + a.minutes * 60
        counts = {n: 0 for n in names}
        refused = {n: 0 for n in names}

        def roam(n):
            while time.time() < stop_at:
                kind, cmds, desc = route_commands(rng)
                for c in cmds:
                    link.send_name(n, c)
                    time.sleep(0.05)
                before = watch.started.get(n)
                link.send_name(n, "run demo")
                run = watch.wait_start(n, before, 10)
                if not run:
                    refused[n] += 1
                    if refused[n] in (1, 5):
                        recent = [m for t, who, m in watch.messages[-10:] if who == n]
                        print("  %s didn't start a route%s" % (n, (": " + str(recent[-1])) if recent else ""))
                    time.sleep(2)
                    continue
                counts[n] += 1
                print("  %s: route %d, %s" % (n, counts[n], desc))
                watch.wait_end(run, max(5.0, stop_at - time.time() + 60))
                time.sleep(rng.uniform(0.5, 2.5))     # a pause, so robots are also seen standing still

        threads = [threading.Thread(target=roam, args=(n,), daemon=True) for n in names]
        for th in threads:
            th.start()
        while any(th.is_alive() for th in threads):
            time.sleep(1)
        stop_all()
        print("Routes driven: " + ", ".join("%s %d" % (n, counts[n]) for n in names))

        note("phase objects")
        print("\nNow put things on and off the board by hand: boxes, cups, tools, a hand on its own, "
              "objects next to and touching the robots. Move a robot by hand now and then.")
        input("Press Enter when you're done ... ")
        note("phase end")
        stop_all()
        print("\nDone. Stop the Pi recorder now (Ctrl+C in its window). Robot logs: %s" % outdir)
    except KeyboardInterrupt:
        print("\nStopping every robot ...")
        stop_all()
        note("stopped early")
    finally:
        link.stop.set()


if __name__ == "__main__":
    main()
