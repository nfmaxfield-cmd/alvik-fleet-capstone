#!/usr/bin/env python3
"""Light patterns in an overhead recording -> lights_detections.csv: for every
frame, every pattern seen, which robot it is (or none), how sure, where its
LEDs' centre is and which way it faces. The optional second identity source
next to tools/blink/blink.py's blink codes (patterns: patterns.py; how it
reads them: detect.py; design note: tools/blink/README.md "Light patterns").

    python tools/lights/lights.py recordings/20261019-101455_E1
    python tools/lights/lights.py <recording> --kind l3 --names Left=S0L,Center=S0R,Right=S0L/0011
    python tools/lights/lights.py <recording> --kind strip --names Left=1,Center=2,Right=3

Needs the recording's floor_calibration.json (beside it or --calib): the
pattern is matched in floor centimetres. tools/blink/blink.py --lights l3
runs this after the blink pass.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, TOOLS, os.path.join(TOOLS, "live"), os.path.join(TOOLS, "blink"), os.path.join(TOOLS, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)

import patterns as PT  # noqa: E402
import detect as D  # noqa: E402

SOURCE = "lights"


def out_cols(names):
    return (["frame", "t_wall", "robot", "x_px", "y_px", "heading_img_deg", "code", "score", "track", "kind",
             "n_leds", "off_fwd_cm", "off_left_cm", "led_height_cm"] + ["p_" + n for n in names]
            + ["p_none", "source"])


def make_family(kind, wide=False, family_json=None):
    kw = dict(PT.WIDE_L3) if (wide and kind == "l3") else {}
    if family_json:
        kw.update(json.loads(family_json))
    if "side_cm" in kw:
        kw["side_cm"] = tuple(kw["side_cm"])
    return PT.family(kind, **kw)


def make_tracker(fam, calib, names, params=None):
    import floormap as FMAP
    fm = FMAP.FloorMap(calib, led_height_cm=fam.height_cm)
    return D.Tracker(fam, fm, names, params)


def row_out(r, names, fam):
    d = dict(frame=r["frame"], t_wall="%.4f" % r["t"], robot=r["robot"], x_px="%.2f" % r["x_px"],
             y_px="%.2f" % r["y_px"], heading_img_deg="%.2f" % r["heading_img_deg"],
             code=r["code"] if r["robot"] else "", score="%.3f" % r["score"], track=r["track"], kind=r["kind"],
             n_leds=r["n_leds"], off_fwd_cm="%.2f" % r["off_fwd_cm"], off_left_cm="%.2f" % r["off_left_cm"],
             led_height_cm="%.2f" % fam.height_cm, p_none="%.3f" % r["p_none"], source=SOURCE)
    for n in names:
        d["p_" + n] = "%.3f" % r["probs"].get(n, 0.0)
    return d


def analyse(recording, fam, names, calib=None, params=None, out=None, progress=None, max_frames=None):
    """Runs the tracker over the recording's video; writes lights_detections.csv (out, default in the
    recording). Returns (rows written, tracker)."""
    import cv2
    import blink as B
    import floormap as FMAP
    calib = FMAP.find_calibration(recording, calib)
    if not calib:
        raise SystemExit("No floor_calibration.json beside the recording: give --calib")
    tr = make_tracker(fam, calib, names, params)
    if tr.det.note:
        print("note: " + tr.det.note)
    t = B.repair_late_frames(B.read_frame_times(recording))
    cap = cv2.VideoCapture(os.path.join(recording, "video.avi"))
    out = out or os.path.join(recording, "lights_detections.csv")
    n = 0
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=out_cols(list(names)))
        w.writeheader()
        fi = 0
        while True:
            ok, img = cap.read()
            if not ok or fi >= len(t) or (max_frames and fi >= max_frames):
                break
            for r in tr.step(img, float(t[fi]), fi):
                w.writerow(row_out(r, list(names), fam))
                n += 1
            fi += 1
            if progress and fi % 1500 == 0:
                progress(fi, len(t))
    cap.release()
    return n, tr


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", help="recording folder (video.avi, frames.csv)")
    ap.add_argument("--kind", default="l3", choices=("l3", "strip"))
    ap.add_argument("--names", help="name=code list (default: patterns.py's table for the family)")
    ap.add_argument("--wide", action="store_true", help="l3: the longer L (patterns.WIDE_L3), not the printed holder")
    ap.add_argument("--family", help='geometry overrides as JSON, e.g. \'{"corner_cm": [-1.2, -0.7]}\'')
    ap.add_argument("--calib", help="floor_calibration.json (default: beside the recording)")
    ap.add_argument("--name-p", type=float, default=0.9)
    ap.add_argument("--out", help="output CSV (default <recording>/lights_detections.csv)")
    ap.add_argument("--max-frames", type=int)
    args = ap.parse_args(argv)
    fam = make_family(args.kind, args.wide, args.family)
    names = PT.parse_names(args.names, fam)
    p = D.Params(name_p=args.name_p)
    n, tr = analyse(args.recording, fam, names, args.calib, p, args.out,
                    progress=lambda i, k: print("  frame %d / %d" % (i, k)), max_frames=args.max_frames)
    print("%d rows (%d named); names %s; %s" % (n, tr.stats["named_rows"], names, tr.stats))
    return 0


if __name__ == "__main__":
    sys.exit(main())
