#!/usr/bin/env python3
"""e20 camera-trust scenarios on the hard simulator (measured profile): the simulator breaks the camera guesses.

    python3 docs/results/e20-nov12/cam_scenarios.py SCENARIO SEED MINUTES OUT.json

Scenarios (each camera sees every robot on its own; the detector has the same cameras):
  home3     three cameras: "cam-a" steep above (0.4 cm), "cam-b" at a low angle (noise x3 along its depth, a dark patch
            only it has), "cam-c" a third camera at a strange angle (noise x2.5 along 45 deg, 0.5 cm offset). Detector:
            cam-a heading good (15 deg), cam-b heading anywhere (like the real side camera), cam-c heading 40 deg.
  bump      home3, and cam-a is bumped 6 deg at 40 % of the run (its picture turned for the rest of the run).
  swap      home3, and cam-b and cam-c swap names from 30 % to 50 % of the run (each through the other's calibration).
Nothing here touches a robot, a Pi or a camera.
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.join(HERE, "..", "..", "..", "tools", "fleet")
sys.path.insert(0, os.path.abspath(FLEET))
sys.path.insert(0, os.path.abspath(os.path.join(FLEET, "..", "analysis")))
import freeroam_fastsim as F      # noqa: E402
import fastsim_realism as R       # noqa: E402
import tagfree as TF              # noqa: E402

VERSION = "e20-cam-scenarios-2026-11-12a"


def cams_for(sc, T, board):
    B = board
    dark = [(B[0] + 10, B[1] + 10, B[0] + 50, B[1] + 50)]
    cams = [dict(name="cam-a", share=0.93, sd=0.4),
            dict(name="cam-b", share=0.9, sd=0.6, aniso=(3.0, 90.0), dark=dark),
            dict(name="cam-c", share=0.85, sd=0.8, aniso=(2.5, 45.0), bias=(0.5, -0.3))]
    ml = [dict(name="cam-a", sigma=1.6, th_sigma=15.0, recall=0.96),
          dict(name="cam-b", sigma=2.6, th_any=True, recall=0.9, dark=dark),
          dict(name="cam-c", sigma=3.0, th_sigma=40.0, recall=0.85)]
    swap = None
    if sc == "bump":
        cams[0]["bump"] = (0.4 * T, 6.0)
        ml[0]["bump"] = (0.4 * T, 6.0)
    if sc == "swap":
        swap = (0.3 * T, 0.5 * T, "cam-b", "cam-c")
    return cams, ml, swap, dark


def main(sc, seed, mins, out):
    T = mins * 60.0
    prof = R.load_profile("measured")
    Cls = R.make_class(F)
    probe = Cls(seed, 0.01, profile="measured", prof=prof)
    cams, ml, swap, dark = cams_for(sc, T, probe.board)
    del probe
    t0 = time.time()
    s = Cls(seed, mins / 60.0, profile="measured", prof=prof, cams=cams, cam_swap=swap, camera="detections",
            sensors=True, edge_physics=True)
    scene = dict(ml_cams=ml)
    if swap:
        scene["ml_swap"] = swap
    sd = TF.SimDetector(scene, seed)
    sd.sim = s
    TF.attach(s.src.layer, sd, ctrl=s.ctrl, mode="1", say=lambda w: None)
    book = s.src.layer.book
    import camera_trust as CT
    if book is None:                  # MIRLT_CAMERA_TRUST=e19 (the baseline): an empty book, for the summary only
        book = CT.TrustBook("e19-baseline")
    timeline = []
    orig = s.truth

    def truth():
        orig()
        if int(s.t * 10) % 300 == 0 and (not timeline or s.t - timeline[-1]["t"] > 5):
            row = dict(t=round(s.t, 1))
            for c in ("cam-a", "cam-b", "cam-c"):
                sp = book.score(c, "motion", "position", area=None)
                sp["state"] = book.state(c, "motion", "position", area=None)
                sf = book.score(c, "motion", "position-frame", area=None)
                sh = book.score(c, "ml", "heading", area=None)
                sm = book.score(c, "motion-heading", "heading", area=None)
                row[c] = dict(pos=sp["state"], pos_agree=sp["agree"], pos_n=sp["checks"], frame=sf["state"],
                              frame_agree=sf["agree"], frame_n=sf["checks"], ml_head=sh["state"],
                              ml_head_agree=sh["agree"], ml_head_n=sh["checks"], mh=sm["state"], mh_n=sm["checks"])
            # the dark patch, seen by cam-b only as dark
            a = book.area(dark[0][0] + 20, dark[0][1] + 20)
            row["cam-b_dark"] = book.score("cam-b", "motion", "position", area=a, light="dark")["state"]
            timeline.append(row)
    s.truth = truth
    summ = s.run()
    b = s.breakage(summ)
    res = dict(version=VERSION, scenario=sc, seed=seed, minutes=mins, secs=round(time.time() - t0, 1), cams=cams,
               ml_cams=ml, swap=swap, broke={k: b.get(k) for k in ("contacts", "falls", "swaps", "min_gap_cm",
                                                                    "longest_free_stand_s", "hung_s")},
               trust=book.summary(), words=book.plain_words(), timeline=timeline,
               motion_heading=s.ctrl.mheading.summary(), detector=sd.summary(),
               ctrl={k: s.ctrl.counts.get(k) for k in ("motion_headings", "heading_confirmed_by_motion", "hd_settled_by_motion", "hd_confirm_crawls",
                                                        "odo_learn_refused", "blind_checks_refused", "camera_tests")},
               refused=s.src.layer.trust_refused,
               odo={n: t.summary() for n, t in s.ctrl.trust.items()})
    with open(out, "w") as f:
        json.dump(res, f, default=str)
    if os.environ.get("SAVE_BOOK"):            # the book itself (Mission Control's view reads a site's book)
        book.save(os.environ["SAVE_BOOK"], why="scenario %s seed %d" % (sc, seed))
    print(sc, seed, "done in", res["secs"], "s", res["broke"])
    for w in res["words"]:
        print("  ", w)


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
