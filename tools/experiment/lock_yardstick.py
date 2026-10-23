"""Yardstick (Nick, 2026-10-22: "build an off-the-shelf tracker as a yardstick"): OpenCV's CSRT / KCF / MIL trackers
on the evening stretches in tests/data/realclips/oct22eve_*.json. Prints the share of pictures with a place, ms a
picture and the distance from the runner's camera fixes, to set beside tests/test_e02_realclips.py's lock-on numbers.

Needs OpenCV's contrib trackers, kept apart from the main OpenCV:
    pip install --target <dir> opencv-contrib-python-headless==4.10.0.84
and the footage (get_clips.py, or E02_CLIPS pointing at it):
    PYTHONPATH=<dir> python tools/experiment/lock_yardstick.py csrt mil

2026-10-23: CSRT 17 ms a picture, drifted 24 cm off Right without saying so; MIL as close at the fixes as the
lock-on but 67 ms a picture; KCF failed on these pictures (an OpenCV error)."""
import json
import math
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]
import e02_camera as CAM  # noqa: E402
import e02_replay as R  # noqa: E402

DATA = os.path.join(HERE, "tests", "data", "realclips")
CLIPS = os.environ.get("E02_CLIPS") or os.path.join(HERE, "data", "clips")
STRETCHES = ("oct22eve_left", "oct22eve_center", "oct22eve_right")
MAKERS = {"csrt": "TrackerCSRT_create", "kcf": "TrackerKCF_create", "mil": "TrackerMIL_create"}


def run(kind, name, cam, rpx):
    m = json.load(open(os.path.join(DATA, name + ".json")))
    rec = R.MjpgPiece(os.path.join(CLIPS, m["video"]), os.path.join(CLIPS, m["frames"]), 0)
    i0, i1 = rec.index(m["start_t"]), rec.index(m["end_t"])
    tr = getattr(cv2, MAKERS[kind])()
    u, v = cam.project((m["at"][0], m["at"][1], CAM.ROBOT_TOP_CM))
    h = 1.3 * rpx
    tr.init(rec.read(i0), (int(u - h / 2), int(v - h / 2), int(h), int(h)))
    out, tt = [], 0.0
    for i in range(i0, i1 + 1):
        a = time.perf_counter()
        ok, b = tr.update(rec.read(i))
        tt += time.perf_counter() - a
        if ok:
            x, y = cam.to_floor(b[0] + b[2] / 2, b[1] + b[3] / 2, CAM.ROBOT_TOP_CM)
            out.append((rec.t_frame(i), x, y))
        else:
            out.append((rec.t_frame(i), None, None))
    seen = sum(1 for o in out if o[1] is not None)
    errs = []
    for tf, x, y in m["fixes"]:
        near = [o for o in out if o[1] is not None and abs(o[0] - tf) < 0.5]
        errs.append(round(min(math.hypot(o[1] - x, o[2] - y) for o in near), 1) if near else None)
    print(kind, name, "a place in %.0f%% of pictures," % (100.0 * seen / len(out)), "%.1f ms a picture," % (
        1000 * tt / len(out)), "cm from the fixes:", errs, flush=True)


def main():
    cam = CAM.load_camera(os.path.join(HERE, "data", "overhead_floorcal_roam1.json"))
    rpx = CAM.ROBOT_CM * cam.px_per_cm(61, 61, CAM.ROBOT_TOP_CM)
    for kind in sys.argv[1:] or ["csrt", "mil"]:
        if not hasattr(cv2, MAKERS[kind]):
            print(kind, "not in this OpenCV (needs opencv-contrib)")
            continue
        for name in STRETCHES:
            try:
                run(kind, name, cam, rpx)
            except cv2.error as e:
                print(kind, name, "failed:", str(e).splitlines()[-1])


if __name__ == "__main__":
    main()
