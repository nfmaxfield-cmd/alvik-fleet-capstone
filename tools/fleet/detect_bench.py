#!/usr/bin/env python3
"""Tag detection speed, bright against dim, and the cheap speed-ups (build e14; school 17:46: in the dim room the tag
bridge's detection time per picture climbed from 122 to 421 ms; a standing tag got 2.3 fixes a second).

Runs the tag bridge's own tracker (tagcore.WindowTracker: small windows round each tag's last spot, the whole picture
every FULL_EVERY pictures while a wanted tag is missing, then a downscaled retry) on the school camera's simulated
pictures, and reports per variant: the median and 95th-percentile ms a picture, the share of pictures each tag was read
in, and where the time went (windows / whole-picture searches). Variants (each is a switch in tag_bridge, off unless the
measurement says otherwise):
  base         as the bridge ran before e14 (every window allows the full speed; each other variant adds one change)
  dim_noup     in a dim picture, the whole-picture search without its 2x enlargement
  dim_half     in a dim picture, the whole-picture search on a half-size picture (decimated) instead
  backoff      a tag missing for 10 whole-picture searches is looked for every 3 x FULL_EVERY pictures
  threads1     OpenCV on one thread (cv2.setNumThreads(1): what a throttled Pi might prefer)
  dim_blur     in a dim picture, each search area smoothed (Gaussian, 1 px) first: fewer noise contours
  big_noup     a window round a tag already 24 px or bigger searched without its 2x enlargement
  measured_roi each window's speed allowance from the tag's own measured speed (tagcore MIRLT_TAG_ROI=measured, the
               bridge's default from e14)
--interval S: the time between pictures (the school's dim run: about 0.43 s; lit about 0.1 s): a window allows for
the robot's travel since the last read, so slow pictures make bigger windows.
--frames DIR: real pictures (*.jpg / *.png) instead of the simulator's (e.g. saved from the school camera).
Prints a table; --json writes it. Never touches a camera.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (HERE, os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.append(p)

import cv2              # noqa: E402
import tagcore as TC    # noqa: E402

VERSION = "detect-bench-2026-11-10e14"
VARIANTS = ("base", "dim_noup", "dim_half", "backoff", "threads1", "dim_blur", "big_noup", "measured_roi")


def sim_pictures(n, light, seed=1, tags=((4, 64.0, 116.0), (5, 90.0, 84.0), (6, 110.0, 60.0), (2, 130.0, 100.0)),
                 noise=None):
    """The school camera (behind the TV end, 145 cm up, looking down the table; 1280 x 960) and roof tags standing
    still. light: 1 = the room lit; 0.3 = the TV light alone (gain and noise as a camera's auto-exposure gives in dim
    light: the picture darker and grainier)."""
    import tag_cam as TCAM
    cam = TCAM.TagCamera((-114.3, 83.8, 145.0), (150.0, 83.8, 0.0), size=(1280, 960), hfov_deg=70.4, seed=seed,
                         gain=float(light), noise=float(noise if noise is not None else 3.0 / max(0.2, light) ** 0.5),
                         table=(0, 0, 396.2, 167.6))
    rs = [dict(x=x, y=y, th=math.radians(90.0), z=6.0, family=TC.TAG_FAMILY, id=i, tag_cm=8.0, roof_cm=9.5)
          for i, x, y in tags]
    return [cam.render(rs, t=k / 7.0) for k in range(n)]


def file_pictures(folder, n):
    out = []
    for p in sorted(glob.glob(os.path.join(folder, "*.jpg")) + glob.glob(os.path.join(folder, "*.png")))[:n]:
        g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if g is not None:
            out.append(g)
    return out


class Variant:
    """The tracker with one speed-up applied (the same code paths the bridge's switches use)."""

    def __init__(self, name, ids, dim_grey=50.0):
        self.name, self.dim_grey = name, dim_grey
        self.det = TC.TagDetector(TC.TAG_FAMILY, ids=list(ids))
        self.trk = TC.WindowTracker(self.det, list(ids))
        self.missing_fulls = 0
        self._roi = TC.ROI_RULE

    def step(self, g, t):
        dim = float(g[::8, ::8].mean()) < self.dim_grey
        trk, det = self.trk, self.det
        orig_full = det.detect_full
        if self.name == "dim_noup" and dim:
            det.detect_full = lambda grey, cam=None, t=None, upscale=None: orig_full(grey, cam=cam, t=t, upscale=1.0)
        elif self.name == "dim_half" and dim:
            def half(grey, cam=None, t=None, upscale=None):
                small = cv2.resize(grey, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
                return det.detect(small, cam=cam, t=t, scale=0.5)
            det.detect_full = half
        orig_detect, orig_win = det.detect, det.detect_window
        if self.name == "dim_blur" and dim:
            det.detect = lambda grey, cam=None, t=None, scale=1.0, offset=(0.0, 0.0): orig_detect(
                cv2.GaussianBlur(grey, (0, 0), 1.0), cam=cam, t=t, scale=scale, offset=offset)
        if self.name == "big_noup":
            def win(grey, cx, cy, half_px, cam=None, t=None, upscale=None):
                big = any(abs(cx - l[0]) < 1e-6 and l[2] >= 24.0 for l in trk.last.values())
                return orig_win(grey, cx, cy, half_px, cam=cam, t=t, upscale=1.0 if big else upscale)
            det.detect_window = win
        # every other variant is measured against the tracker as it was before e14 (the full speed allowance)
        TC.ROI_RULE = "measured" if self.name == "measured_roi" else "max"
        if self.name == "backoff":
            trk.full_every = TC.FULL_EVERY * 3 if self.missing_fulls >= 10 else TC.FULL_EVERY
        f0 = trk.fulls
        t0 = time.perf_counter()
        found = trk.step(g, t)
        ms = (time.perf_counter() - t0) * 1000.0
        det.detect_full = orig_full
        det.detect, det.detect_window = orig_detect, orig_win
        TC.ROI_RULE = self._roi
        if trk.fulls > f0:
            self.missing_fulls = self.missing_fulls + 1 if [i for i in trk.ids if i not in found] else 0
        return found, ms, trk.fulls > f0


def run(pics, ids, variant, interval=1 / 7.0):
    old = cv2.getNumThreads()
    if variant == "threads1":
        cv2.setNumThreads(1)
    try:
        v = Variant(variant, ids)
        ms, fulls_ms, win_ms, seen = [], [], [], {i: 0 for i in ids}
        for k, g in enumerate(pics):
            found, m, full = v.step(g, k * interval)
            ms.append(m)
            (fulls_ms if full else win_ms).append(m)
            for i in found:
                seen[i] += 1
    finally:
        cv2.setNumThreads(old)
    n = max(1, len(pics))
    return dict(variant=variant, pictures=len(pics), median_ms=round(float(np.median(ms)), 1),
                p95_ms=round(float(np.percentile(ms, 95)), 1), mean_ms=round(float(np.mean(ms)), 1),
                window_pictures_ms=round(float(np.median(win_ms)), 1) if win_ms else None,
                full_search_pictures_ms=round(float(np.median(fulls_ms)), 1) if fulls_ms else None,
                full_searches=len(fulls_ms), read_share={str(i): round(c / n, 3) for i, c in seen.items()})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=60, help="pictures per light and variant")
    ap.add_argument("--lights", default="1.0,0.3", help="simulated light levels (1 = lit)")
    ap.add_argument("--ids", default="4,5,6,2,7", help="the wanted tag ids (7: one not on the table, as a robot away)")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--interval", type=float, default=1 / 7.0, help="seconds between pictures")
    ap.add_argument("--frames", default="", help="a folder of real pictures instead of the simulator's")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    sets = {"files": file_pictures(a.frames, a.n)} if a.frames else {
        "light %s" % L: sim_pictures(a.n, float(L)) for L in a.lights.split(",")}
    rows = []
    for label, pics in sets.items():
        grey = float(np.mean([p[::8, ::8].mean() for p in pics])) if pics else 0.0
        for v in a.variants.split(","):
            r = run(pics, ids, v, a.interval)
            r.update(set=label, interval_s=a.interval, grey=round(grey, 1), cv2_threads=cv2.getNumThreads())
            rows.append(r)
            print("%-10s grey %5.1f  %-9s median %6.1f ms  p95 %6.1f  windows %6s  whole %6s (%2d)  read %s" % (
                label, grey, v, r["median_ms"], r["p95_ms"], r["window_pictures_ms"], r["full_search_pictures_ms"],
                r["full_searches"], r["read_share"]), flush=True)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(dict(version=VERSION, rows=rows), f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
