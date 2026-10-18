#!/usr/bin/env python3
"""The live link: overhead camera -> robot positions -> run_plan.py.

Finds each robot by its blinking top lights (which robot = the blink rhythm,
where = the two LEDs), turns that into the robot's floor position and heading
with floor_calibration.json, and sends it to tools/fleet/run_plan.py's camera
port as it happens.

    # on the laptop, while camrec.py runs on the Pi:
    python tools/live/track_live.py --stream st1maxfini.local:8000 --calib <folder>/floor_calibration.json
    python tools/fleet/run_plan.py --map map.json --plan plan.json --fixes udp

    # without the robots: play a recording back in real time
    python tools/live/track_live.py --replay recordings/20261018-101455_E1 --dry-run
    python tools/live/track_live.py --replay sim_out/recording --to 127.0.0.1:5010

Sends one JSON object per UDP packet to --to (default 127.0.0.1:5010, run_plan's
--fixes udp port), as docs/data-formats.md "Position source" describes:
  {"type": "fix", "robot": "Left", "t": <laptop time the picture was taken>,
   "x": cm, "y": cm, "th": deg, "sigma": cm, "sigma_th": deg,
   "id_p": <identity probability>, "path": "fast" | "late", "t_pi", "frame", "track"}
run_plan.py uses robot, t, x, y, th, sigma, sigma_th and ignores the rest.
A fix is sent only when the light's name is sure (see tools/live/livecore.py);
lights that aren't sure are logged as "sighting" (sent too with --sightings;
run_plan ignores them).

Clock: t = Pi t_wall + (laptop - Pi clock offset) - --capture-delay-ms. The
offset is measured continuously from the stream itself (see sources.py); the
capture delay (picture taken -> Pi stamps it) is one frame unless measured
(tools/live/capture_delay.py).

Writes --log (default logs/<date-time>-live/): messages.jsonl (everything
sent or held back, with t_sent), clock.csv (offset every second),
live_summary.json (fixes per robot, rates, latency, processing time).

Obstacles: not detected yet. obstacles.py is the hook (see there).
"""
import argparse
import csv
import datetime as dt
import json
import os
import socket
import sys
import time
from collections import defaultdict, deque

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "blink"), os.path.join(TOOLS, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import blink as B  # noqa: E402
import floormap as FMAP  # noqa: E402
import livecore as LC  # noqa: E402
import obstacles as OB  # noqa: E402
import sources as S  # noqa: E402


def pct(v, q):
    return round(float(np.percentile(v, q)), 1) if len(v) else None


def led_offsets_from(path):
    import runs as FR
    rows = FR.load_calib_results(path, 0.0)
    offs, src = FR.led_offsets(rows)
    return offs, src


class Sender:
    def __init__(self, to, dry_run=False, out=print):
        self.dry = dry_run
        self.out = out
        self.sock = None
        if not dry_run:
            host, _, port = to.rpartition(":")
            self.addr = (host or "127.0.0.1", int(port))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg):
        data = json.dumps(msg, separators=(",", ":"))
        if self.dry:
            self.out(data)
        else:
            try:
                self.sock.sendto(data.encode(), self.addr)
            except OSError as e:
                self.out("send failed: %s" % e)

    def close(self):
        if self.sock:
            self.sock.close()


def make_tracker(args, calib_path):
    fmap = FMAP.FloorMap(calib_path, led_height_cm=args.led_height_cm)
    names = B.parse_names(args.names)
    p = B.Params(names=names, px_per_cm=fmap.px_per_cm(), vmax_cms=args.vmax_cms)
    if args.exposure_ms:
        p.exposure_s = args.exposure_ms / 1000.0
    elif args.replay:
        p.exposure_s = B.exposure_from_meta(args.replay)[0]
    lp = LC.LiveParams(name_p=args.name_p, fast=not args.no_fast)
    if args.calib_results:
        offs, src = led_offsets_from(args.calib_results)
        lp.led_offset = offs
        print("light offsets from %s: %s" % (src, {k or "others": tuple(round(v, 2) for v in o)
                                                  for k, o in offs.items()}))
    if args.led_offset_cm:
        f, l = (float(v) for v in args.led_offset_cm.split(","))
        lp.led_offset = {None: (f, l)}
    return LC.LiveTracker(fmap, names, p, lp), fmap


def run(args, source, sender, log_dir, out=print):
    calib = FMAP.find_calibration(args.replay, args.calib)
    if not calib:
        raise SystemExit("No floor_calibration.json: give --calib")
    tracker, fmap = make_tracker(args, calib)
    if fmap.note:
        out("note: " + fmap.note)
    out("floor calibration %s; %.2f px per cm; names %s" % (calib, tracker.p.px_per_cm, tracker.p.names))
    obst = OB.make(args.obstacles)
    os.makedirs(log_dir, exist_ok=True)
    mf = open(os.path.join(log_dir, "messages.jsonl"), "w")
    cf = open(os.path.join(log_dir, "clock.csv"), "w", newline="")
    cw = csv.writer(cf)
    cw.writerow(["t_laptop", "t_pi", "offset_s", "capture_delay_s"])
    clock = S.ClockEnvelope()
    arrived = {}                       # frame index -> laptop arrival time
    proc_ms, ages = [], defaultdict(list)
    per_robot = defaultdict(lambda: dict(fixes=0, fast=0, late=0, first=None, last=None, id_p_min=1.0))
    intervals = deque(maxlen=50)
    last_tpi = None
    t_status = time.time()
    n_status = 0
    capture = None if args.capture_delay_ms is None else args.capture_delay_ms / 1000.0
    replay_offset = None
    t_start = time.time()
    try:
        for fr in source:
            if last_tpi is not None and fr.t_pi > last_tpi:
                intervals.append(fr.t_pi - last_tpi)
            last_tpi = fr.t_pi
            clock.add(fr.t_arrived, fr.t_pi)
            if args.replay:
                # a replay has no network: its clock mapping is fixed by the first frame
                if replay_offset is None:
                    replay_offset = fr.t_arrived - fr.t_pi
                offset = replay_offset
            else:
                offset = clock.offset
            cap_s = capture if capture is not None else (float(np.median(intervals)) if len(intervals) >= 5 else 0.04)
            arrived[fr.index] = fr.t_arrived
            arrived.pop(fr.index - 200, None)
            t0 = time.perf_counter()
            msgs = tracker.push(fr.img, fr.t_pi)
            for o in obst.update(fr.t_pi, fr.img, tracker) if obst else []:
                msgs.append(o)
            proc_ms.append((time.perf_counter() - t0) * 1000.0)
            for m in msgs:
                m["t"] = round(m["t_pi"] + offset - cap_s, 4)
                now = time.time()
                if m["type"] == "fix" or (m["type"] == "sighting" and args.sightings) or m["type"] == "obstacle":
                    sender.send(m)
                    m["sent"] = True
                m["t_sent"] = round(now, 4)
                a = arrived.get(m.get("frame"))
                if a is not None:
                    m["proc_ms"] = round((now - a) * 1000.0, 2)
                mf.write(json.dumps(m, separators=(",", ":")) + "\n")
                if m["type"] == "fix":
                    r = per_robot[m["robot"]]
                    r["fixes"] += 1
                    r[m["path"]] += 1
                    r["first"] = m["t"] if r["first"] is None else r["first"]
                    r["last"] = m["t"]
                    r["id_p_min"] = min(r["id_p_min"], m["id_p"])
                    ages[m["path"]].append((now - m["t"]) * 1000.0)
            now = time.time()
            if now - t_status >= 1.0:
                cw.writerow(["%.4f" % now, "%.4f" % fr.t_pi, "%.5f" % offset, "%.4f" % cap_s])
                n_status += 1
                if not args.quiet and n_status % 2 == 0:
                    out(status_line(tracker, per_robot, proc_ms, fr, now - t_start))
                t_status = now
    except KeyboardInterrupt:
        out("stopped")
    finally:
        mf.close()
        cf.close()
    summ = summary(tracker, per_robot, proc_ms, ages, time.time() - t_start, getattr(source, "gaps", 0))
    with open(os.path.join(log_dir, "live_summary.json"), "w") as f:
        json.dump(summ, f, indent=1)
    return summ


def status_line(tracker, per_robot, proc_ms, fr, el):
    names = ", ".join("%s %d" % (n, r["fixes"]) for n, r in sorted(per_robot.items())) or "none yet"
    tr = "; ".join("%s:%s" % (d["track"], d["name"] or d["why"]) for d in tracker.summary_tracks()[:6])
    return "%5.0f s  frame %d  %.1f ms/frame  fixes: %s  lights: %s" % (
        el, fr.pi_frame, float(np.mean(proc_ms[-50:])) if proc_ms else 0.0, names, tr or "none")


def summary(tracker, per_robot, proc_ms, ages, elapsed, gaps):
    robots = {}
    for n, r in per_robot.items():
        span = (r["last"] - r["first"]) if r["first"] is not None else 0.0
        robots[n] = dict(fixes=r["fixes"], fast=r["fast"], late=r["late"], span_s=round(span, 1),
                         fixes_per_s=round(r["fixes"] / span, 2) if span > 0 else None, id_p_min=r["id_p_min"])
    return dict(frames=tracker.stats["frames"], elapsed_s=round(elapsed, 1), frames_missing=gaps,
                processing_ms=dict(median=pct(proc_ms, 50), p95=pct(proc_ms, 95), max=pct(proc_ms, 100)),
                fix_age_ms={k: dict(median=pct(v, 50), p95=pct(v, 95), n=len(v)) for k, v in ages.items()},
                robots=robots, tracker=tracker.stats)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--stream", help="camrec.py on the Pi, e.g. st1maxfini.local:8000 (reads /live)")
    src.add_argument("--replay", help="a recording folder (video.avi, frames.csv) played back in real time")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed (0 = as fast as possible)")
    ap.add_argument("--start-s", type=float, default=0.0, help="replay: skip this much of the recording")
    ap.add_argument("--max-s", type=float, default=0.0, help="replay: stop after this much (0 = all)")
    ap.add_argument("--calib", help="floor_calibration.json (replay: found beside the recording)")
    ap.add_argument("--calib-results", help="calibration_results.csv (tools/blink/calib.py): each robot's light offset")
    ap.add_argument("--led-offset-cm", help="light offset from the turning centre, forward,left (default 2.75,0)")
    ap.add_argument("--led-height-cm", type=float, default=FMAP.LED_HEIGHT_CM, help="height of the top lights (4)")
    ap.add_argument("--names", help="blink codes, e.g. 1=Left,2=Center,3=Right,4=other (the default)")
    ap.add_argument("--name-p", type=float, default=0.9, help="identity probability needed before fixes are sent")
    ap.add_argument("--vmax-cms", type=float, default=40.0)
    ap.add_argument("--exposure-ms", type=float, help="camera exposure (replay: from meta.json)")
    ap.add_argument("--capture-delay-ms", type=float, help="picture taken -> Pi time stamp (default one frame)")
    ap.add_argument("--to", default="127.0.0.1:5010", help="where run_plan.py listens (--fixes udp[:PORT])")
    ap.add_argument("--dry-run", action="store_true", help="print the messages instead of sending them")
    ap.add_argument("--sightings", action="store_true", help="also send lights that aren't sure (type sighting)")
    ap.add_argument("--no-fast", action="store_true", help="only send fixes once each flash has ended")
    ap.add_argument("--obstacles", default="none", help="obstacle finder (obstacles.py; only 'none' so far)")
    ap.add_argument("--log", help="log folder (default logs/<date-time>-live)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    log_dir = args.log or os.path.join("logs", dt.datetime.now().strftime("%Y%m%d-%H%M%S") + "-live")
    if args.replay:
        source = S.ReplaySource(args.replay, speed=args.speed, start_s=args.start_s, max_s=args.max_s)
    else:
        if not args.calib:
            raise SystemExit("Give --calib <floor_calibration.json> (made for this camera position)")
        source = S.StreamSource(args.stream)
    sender = Sender(args.to, args.dry_run)
    print("Sending to %s" % ("this window (dry run)" if args.dry_run else args.to))
    try:
        summ = run(args, source, sender, log_dir)
    finally:
        sender.close()
        source.close()
    print(json.dumps(summ["robots"], indent=1))
    print("processing %s ms per frame; fix age (picture taken -> sent) %s; saved to %s"
          % (summ["processing_ms"], summ["fix_age_ms"], log_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
